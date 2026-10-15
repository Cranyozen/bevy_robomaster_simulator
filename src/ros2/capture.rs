//! ROS2 图像捕获实现

use crate::capture::{
    CameraFov, CaptureBundle, ImageHandle, compute_camera_intrinsics,
    driver::{
        CaptureConfig, CaptureFrameId, CapturedFrame, CapturedFrameKind, GpuCaptureHandler,
        SnapshotAsync, SnapshotSync,
    },
    setup_capture_camera, setup_preview_window, sync_capture_camera,
};
use crate::ros2::image::{compress_image, rgb_resized};
use crate::ros2::prelude::WallRateLimiter;
use crate::ros2::topic::{CameraInfoTopic, ImageCompressedTopic, ImageRawTopic, TopicPublisher};
use crate::systems::GameplaySystems;
use bevy::ecs::world::DeferredWorld;
use bevy::prelude::*;
use bevy::render::{RenderApp, RenderSystems};
use r2r::Clock;
use r2r::sensor_msgs::msg::{CameraInfo, CompressedImage, RegionOfInterest};
use r2r::std_msgs::msg::Header;
use std::borrow::Cow;
use std::cell::RefCell;
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, SyncSender};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Everything that decides *whether* a captured frame becomes a message.
///
/// All three live behind one shared handle because `RosCaptureContext` is cloned
/// into the render world: the gates must be the same objects on both sides, not
/// two copies that each think they published recently.
pub struct CaptureGates {
    /// Rate gate for the image stream (`[capture.color] publish_hz`).
    image: Mutex<WallRateLimiter>,
    /// Rate gate for `/camera_info` (`[capture.color] camera_info_hz`).
    camera_info: Mutex<WallRateLimiter>,
    /// Captures accepted but not yet published. Each one holds a GPU readback
    /// and a worker slot for ~100 ms, most of which is spent waiting for
    /// `map_async` to complete on a later device poll. One in flight is what
    /// capped the whole loop at ~6 images/s, so a few are allowed to overlap;
    /// the bound is what keeps that from turning into latency.
    in_flight: AtomicUsize,
    max_in_flight: usize,
}

impl CaptureGates {
    pub fn new(
        image_period: Option<Duration>,
        camera_info_period: Option<Duration>,
        max_in_flight: usize,
    ) -> Self {
        Self {
            image: Mutex::new(WallRateLimiter::from_period(image_period)),
            camera_info: Mutex::new(WallRateLimiter::from_period(camera_info_period)),
            in_flight: AtomicUsize::new(0),
            max_in_flight: max_in_flight.max(1),
        }
    }

    /// Claims one of the bounded capture slots, or `None` when they are all busy.
    fn try_acquire(self: &Arc<Self>) -> Option<CaptureSlot> {
        let mut current = self.in_flight.load(Ordering::Acquire);
        loop {
            if current >= self.max_in_flight {
                return None;
            }
            match self.in_flight.compare_exchange_weak(
                current,
                current + 1,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return Some(CaptureSlot(self.clone())),
                Err(observed) => current = observed,
            }
        }
    }

    #[cfg(test)]
    fn in_flight(&self) -> usize {
        self.in_flight.load(Ordering::Acquire)
    }
}

/// Releases a capture slot when the frame is published — or dropped without ever
/// reaching a worker.
pub(crate) struct CaptureSlot(Arc<CaptureGates>);

impl Drop for CaptureSlot {
    fn drop(&mut self) {
        self.0.in_flight.fetch_sub(1, Ordering::AcqRel);
    }
}

/// Decides, once per rendered frame, whether this frame is worth reading back.
///
/// Returning `None` here is what makes `publish_hz` cheap: `image_copy_driver`
/// skips the whole copier for that frame, so a skipped frame costs no GPU->CPU
/// copy, no BGRA->RGB repack and no JPEG encode. Before this, every rendered
/// frame paid all three at 1440x1080 and the consumer had to keep up or fall
/// behind.
struct RosSnapshotSync {
    stamp: RefCell<r2r::builtin_interfaces::msg::Time>,
    want_camera_info: bool,
    slot: Option<CaptureSlot>,
}

impl SnapshotSync for RosSnapshotSync {
    fn captured(
        self: Box<Self>,
        world: &mut DeferredWorld,
        _config: &CaptureConfig,
    ) -> Box<dyn SnapshotAsync> {
        Box::new(RosSnapshot {
            stamp: self.stamp,
            want_camera_info: self.want_camera_info,
            slot: self.slot,
            ctx: world.resource::<RosCaptureContextShared>().0.clone(),
        })
    }
}

struct RosSnapshot {
    stamp: RefCell<r2r::builtin_interfaces::msg::Time>,
    want_camera_info: bool,
    /// Held until this frame has been published; dropping it frees the slot.
    slot: Option<CaptureSlot>,
    ctx: Arc<RosCaptureContext>,
}

impl SnapshotAsync for RosSnapshot {
    fn captured(&mut self, frame: CapturedFrame<'_>) {
        if frame.kind != CapturedFrameKind::Rgb8 {
            return;
        }

        // Kept alive for the whole body, then moved into the encode request (it
        // is released when the frame is actually published, not before).
        let want_image = self.slot.is_some();

        let (width, height, data) = match rgb_resized(
            frame.width,
            frame.height,
            frame.data,
            self.ctx.published_size.0,
            self.ctx.published_size.1,
        ) {
            Some((w, h, resized)) => (w, h, Cow::Owned(resized)),
            None => (frame.width, frame.height, Cow::Borrowed(frame.data)),
        };

        let optical_frame_hdr = Header {
            stamp: self.stamp.take(),
            frame_id: "camera_optical_frame".to_string(),
        };

        if self.want_camera_info {
            self.ctx.camera_info.publish(ros_camera_info(
                optical_frame_hdr.clone(),
                width,
                height,
                self.ctx.fov_y,
            ));
        }

        if want_image {
            // The encode happens on the pool, so this thread only pays for the
            // copy that decouples the pixels from the recycled readback buffer
            // (~0.4 ms for a full-resolution frame). The frame's messages are
            // published by the pool's publisher thread, in capture order.
            self.ctx.encode.submit(EncodeRequest {
                seq: 0, // assigned by the pool, in submission order
                header: optical_frame_hdr,
                width,
                height,
                data: data.into_owned(),
                want_raw: self.ctx.publish_raw,
                want_compressed: self.ctx.publish_compressed,
                jpeg_quality: self.ctx.jpeg_quality,
                slot: self.slot.take(),
            });
        }
    }
}

#[derive(Default)]
struct RosSnapshotCreator {}

impl GpuCaptureHandler for RosSnapshotCreator {
    fn captured(
        &self,
        world: &World,
        _frame_id: Option<CaptureFrameId>,
    ) -> Option<Box<dyn SnapshotSync>> {
        let ctx = world.resource::<RosCaptureContext>();
        let gates = &ctx.gates;

        // The intrinsics are constant while the resolution is, and the detector
        // latches the first message it receives; repeating them per frame is
        // pure traffic.
        let want_camera_info = gates.camera_info.lock().unwrap().allow();

        // Both gates must open: the rate cap decides *how often* a frame is
        // worth reading back, the slot bound decides whether we may start one now.
        let slot = if ctx.wants_image() && gates.image.lock().unwrap().allow() {
            gates.try_acquire()
        } else {
            None
        };

        if slot.is_none() && !want_camera_info {
            return None;
        }

        Some(Box::new(RosSnapshotSync {
            stamp: RefCell::new(Clock::to_builtin_time(
                &ctx.clock.lock().unwrap().get_now().unwrap(),
            )),
            want_camera_info,
            slot,
        }))
    }
}

#[derive(Resource, Clone, Deref, DerefMut)]
pub struct RosCaptureContextShared(Arc<RosCaptureContext>);

/// One captured frame on its way to the wire, handed to the encoder pool.
struct EncodeRequest {
    seq: u64,
    header: Header,
    width: u32,
    height: u32,
    /// Published-size `rgb8` pixels. Owned, not borrowed: the capture driver
    /// recycles the readback buffer as soon as the snapshot returns, so anything
    /// that outlives the callback has to carry its own copy.
    data: Vec<u8>,
    want_raw: bool,
    want_compressed: bool,
    jpeg_quality: u8,
    /// Kept alive until the frame is published, so `max_in_flight` still bounds
    /// the whole pipeline, not just the readback half of it.
    slot: Option<CaptureSlot>,
}

/// An encoded frame waiting for its turn on the wire. `pub(crate)` fields: the
/// pool hands these to the publisher closure, which the ROS plugin owns.
pub(crate) struct EncodedFrame {
    pub(crate) seq: u64,
    pub(crate) raw: Option<r2r::sensor_msgs::msg::Image>,
    pub(crate) compressed: Option<CompressedImage>,
    pub(crate) slot: Option<CaptureSlot>,
}

/// Parallel JPEG encode with strictly ordered publication.
///
/// WHY: every stage of a captured frame used to run on the single
/// `capture-readback` thread — map wait, BGRA->RGB repack, optional box
/// downscale, JPEG encode, publish. The map wait is the only slow part that is
/// *waiting* (it overlaps across slots), but `image`'s JPEG encoder is scalar
/// (measured ~21 Mpix/s: ~47 ms for one 1440x1080 frame), so that one thread sat
/// at 100% of a core and pinned the image rate at ~21 Hz while the render threads
/// and the rest of the machine idled. Moving the encode onto a small pool lifts
/// the ceiling to roughly `threads x` that; the detector then gets full-resolution
/// frames at 30-40 Hz instead of half-resolution frames at the same rate.
///
/// WHY THE REORDER BUFFER: the detector hands header stamps to an EKF, so a frame
/// that overtakes an older one injects a stale pose. Encoders finish out of order
/// (that is the whole point), so results are buffered by sequence number and
/// published in capture order. Sequence numbers are assigned by the single
/// readback thread as it submits, so "capture order" is exact.
///
/// WHY A DEDICATED PUBLISHER THREAD: the reorder buffer cannot be drained from
/// the capture callback. A frame holds its capture slot until it is published, so
/// once `max_in_flight` frames are in the pool the capture side stops calling the
/// callback — draining there would deadlock (measured: exactly the frames that
/// filled the slots went unpublished, ~1 image/s). The publisher thread is the
/// only place that both frees slots and is not gated by them.
///
/// Backpressure: the job channel is bounded by the capture-slot count, and a slot
/// is held until its frame is published, so the pool can never run ahead of the
/// in-flight budget.
pub struct EncodePool {
    jobs: SyncSender<EncodeRequest>,
    next_seq: AtomicU64,
}

impl EncodePool {
    /// Spawns `threads` encoders plus one ordered publisher.
    pub fn new(
        threads: usize,
        capacity: usize,
        publish: impl Fn(EncodedFrame) + Send + 'static,
    ) -> Self {
        let (jobs, job_rx) = std::sync::mpsc::sync_channel::<EncodeRequest>(capacity.max(1));
        let (result_tx, result_rx) = std::sync::mpsc::channel::<EncodedFrame>();
        let job_rx = Arc::new(Mutex::new(job_rx));

        for index in 0..threads.max(1) {
            let job_rx = job_rx.clone();
            let result_tx = result_tx.clone();
            std::thread::Builder::new()
                .name(format!("jpeg-encode-{index}"))
                .spawn(move || encode_loop(job_rx, result_tx))
                .expect("failed to spawn the jpeg encode thread");
        }
        drop(result_tx);

        std::thread::Builder::new()
            .name("ros-publish".to_string())
            .spawn(move || publish_loop(result_rx, publish))
            .expect("failed to spawn the ros publish thread");

        Self {
            jobs,
            next_seq: AtomicU64::new(0),
        }
    }

    /// Hands a frame to the pool, returning its sequence number. Blocks when the
    /// encoders are behind — the pressure belongs on the capture side, which
    /// still holds the slot, not on an unbounded queue.
    fn submit(&self, mut request: EncodeRequest) -> u64 {
        let seq = self.next_seq.fetch_add(1, Ordering::Relaxed);
        request.seq = seq;
        let _ = self.jobs.send(request);
        seq
    }
}

fn encode_loop(
    jobs: Arc<Mutex<Receiver<EncodeRequest>>>,
    results: std::sync::mpsc::Sender<EncodedFrame>,
) {
    loop {
        // `recv` is serialized across the pool (mpsc has one queue), but the
        // encode itself — the expensive part — runs outside the lock.
        let job = { jobs.lock().unwrap().recv() };
        let Ok(job) = job else { return };

        let compressed = job.want_compressed.then(|| {
            compress_image(
                job.header.clone(),
                job.width,
                job.height,
                &job.data,
                job.jpeg_quality,
            )
        });
        let raw = job
            .want_raw
            .then(|| raw_image_owned(job.header, job.width, job.height, job.data));

        let frame = EncodedFrame {
            seq: job.seq,
            raw,
            compressed,
            slot: job.slot,
        };
        if results.send(frame).is_err() {
            return;
        }
    }
}

/// Publishes results in capture order, never dropping one on the floor: a frame
/// that arrives early waits here until its predecessors are out.
fn publish_loop(results: Receiver<EncodedFrame>, publish: impl Fn(EncodedFrame)) {
    let mut next = 0u64;
    let mut buffered: BTreeMap<u64, EncodedFrame> = BTreeMap::new();

    while let Ok(frame) = results.recv() {
        buffered.insert(frame.seq, frame);
        while let Some(frame) = buffered.remove(&next) {
            publish(frame);
            next += 1;
        }
    }
}

#[derive(Resource, Clone)]
pub struct RosCaptureContext {
    pub clock: Arc<Mutex<Clock>>,
    pub fov_y: f32,
    pub publish_raw: bool,
    pub publish_compressed: bool,
    pub jpeg_quality: u8,
    /// Resolution the published messages describe. Equal to the capture size
    /// unless `[capture.color] scale` downscales, and it is what `/camera_info`
    /// is built from, so PnP sees consistent intrinsics either way.
    pub published_size: (u32, u32),
    pub gates: Arc<CaptureGates>,
    /// JPEG encode + ordered publication. Shared, because the context is cloned
    /// into the render world while there is still exactly one pool.
    pub encode: Arc<EncodePool>,
    pub camera_info: TopicPublisher<CameraInfoTopic>,
    pub image_raw: TopicPublisher<ImageRawTopic>,
    pub image_compressed: TopicPublisher<ImageCompressedTopic>,
}

impl RosCaptureContext {
    pub fn wants_image(&self) -> bool {
        self.publish_raw || self.publish_compressed
    }
}

pub struct RosCapturePlugin {
    pub config: CaptureConfig,
    pub context: RosCaptureContext,
}

impl Plugin for RosCapturePlugin {
    fn build(&self, app: &mut App) {
        let capture = CaptureBundle::color(
            app,
            self.config.clone(),
            vec![Box::new(RosSnapshotCreator::default())],
        );
        let render_target_handle = capture.color_target().unwrap().clone();

        app.add_plugins(capture)
            .insert_resource(ImageHandle(render_target_handle))
            .insert_resource(CameraFov(self.context.fov_y))
            .insert_resource(self.context.clone())
            .add_systems(Startup, setup_capture_camera)
            .add_systems(Startup, setup_preview_window)
            .add_systems(
                Update,
                sync_capture_camera
                    .after(GameplaySystems::Camera)
                    .before(RenderSystems::Render),
            );
        app.sub_app_mut(RenderApp)
            .insert_resource(RosCaptureContextShared(Arc::new(self.context.clone())))
            .insert_resource(self.context.clone());
    }
}

fn raw_image(hdr: Header, width: u32, height: u32, data: &[u8]) -> r2r::sensor_msgs::msg::Image {
    raw_image_owned(hdr, width, height, Vec::from(data))
}

/// Same as [`raw_image`] but takes the pixels by value: on the encoder thread
/// there is nobody left to share the buffer with, and `Vec::from` would be
/// another 4.66 MB copy at full resolution.
fn raw_image_owned(
    hdr: Header,
    width: u32,
    height: u32,
    data: Vec<u8>,
) -> r2r::sensor_msgs::msg::Image {
    r2r::sensor_msgs::msg::Image {
        header: hdr,
        height,
        width,
        encoding: "rgb8".to_string(),
        is_bigendian: 0,
        step: width * 3,
        data,
    }
}

fn ros_camera_info(hdr: Header, width: u32, height: u32, fov_y: f32) -> CameraInfo {
    let intrinsics = compute_camera_intrinsics(width, height, fov_y);

    CameraInfo {
        header: hdr,
        height,
        width,
        distortion_model: "plumb_bob".to_string(),
        d: vec![0.000, 0.000, 0.000, 0.000, 0.000],
        k: vec![
            intrinsics.fx,
            0.0,
            intrinsics.cx,
            0.0,
            intrinsics.fy,
            intrinsics.cy,
            0.0,
            0.0,
            1.0,
        ],
        p: vec![
            intrinsics.fx,
            0.0,
            intrinsics.cx,
            0.0,
            0.0,
            intrinsics.fy,
            intrinsics.cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ],
        r: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        binning_x: 0,
        binning_y: 0,
        roi: RegionOfInterest {
            x_offset: 0,
            y_offset: 0,
            height,
            width,
            do_rectify: true,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn camera_info_intrinsics_follow_the_published_resolution() {
        // Downscaling reuses the same fov, so K must scale with the pixels: that
        // is exactly what keeps PnP's translation estimates unchanged.
        let full = ros_camera_info(
            Header {
                stamp: Default::default(),
                frame_id: "camera_optical_frame".to_string(),
            },
            1440,
            1080,
            0.785,
        );
        let half = ros_camera_info(
            Header {
                stamp: Default::default(),
                frame_id: "camera_optical_frame".to_string(),
            },
            720,
            540,
            0.785,
        );

        assert_eq!(full.width, 1440);
        assert_eq!(half.width, 720);
        assert!((full.k[0] / 2.0 - half.k[0]).abs() < 1e-3, "fx halves");
        assert!((full.k[2] / 2.0 - half.k[2]).abs() < 1e-3, "cx halves");
        assert!((full.k[5] / 2.0 - half.k[5]).abs() < 1e-3, "cy halves");
        assert_eq!(
            compute_camera_intrinsics(720, 540, 0.785).fy,
            half.k[4],
            "the message must carry what the shared helper computes"
        );
    }

    #[test]
    fn gates_start_open_so_the_first_message_is_not_delayed() {
        let gates = CaptureGates::new(
            Some(Duration::from_secs(1)),
            Some(Duration::from_secs(1)),
            1,
        );

        assert!(gates.image.lock().unwrap().allow());
        assert!(
            !gates.image.lock().unwrap().allow(),
            "and then the rate limit applies"
        );
        assert!(gates.camera_info.lock().unwrap().allow());
    }

    #[test]
    fn capture_slots_bound_the_pipeline_and_are_released_on_drop() {
        let gates = Arc::new(CaptureGates::new(None, None, 2));

        let first = gates.try_acquire().expect("first slot");
        let second = gates.try_acquire().expect("second slot");
        assert_eq!(gates.in_flight(), 2);
        assert!(
            gates.try_acquire().is_none(),
            "the bound is what keeps overlapping readbacks from becoming latency"
        );

        drop(first);
        assert_eq!(gates.in_flight(), 1);
        let third = gates.try_acquire().expect("the freed slot is reusable");
        assert_eq!(gates.in_flight(), 2);

        drop(second);
        drop(third);
        assert_eq!(gates.in_flight(), 0);
    }

    #[test]
    fn a_zero_max_in_flight_still_admits_one_capture() {
        let gates = Arc::new(CaptureGates::new(None, None, 0));

        let slot = gates.try_acquire().expect("even a zero bound admits one");
        assert!(gates.try_acquire().is_none());
        drop(slot);
        assert!(gates.try_acquire().is_some(), "and the slot is reusable");
    }
}
