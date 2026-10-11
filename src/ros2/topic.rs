use bevy::prelude::{App, Resource};
use bevy::tasks::futures_lite;
use bevy::tasks::futures_lite::future::block_on;
use r2r::geometry_msgs::msg::PoseStamped;
use r2r::rm_interfaces::msg::GimbalCmd;
use r2r::sensor_msgs::msg::{CameraInfo, CompressedImage, Image, PointCloud2};
use r2r::std_msgs::msg::String as RosString;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::visualization_msgs::msg::Marker;
use r2r::{Node, QosProfile, WrappedTypesupport};
use std::collections::VecDeque;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

/// Bounded, drop-oldest mailbox with exactly one consumer thread.
///
/// Why not a channel: this is a *sensor* stream feeding a control loop, so the
/// only useful message is the newest one. An unbounded (or deep) queue does not
/// lose data — it converts a slow consumer into growing latency, which for the
/// auto-aim loop is strictly worse than dropping frames. The old implementation
/// used a 1024-deep `futures::mpsc` channel and spawned a task per `publish`,
/// i.e. up to 1024 x 4.66 MB of already-stale images waiting behind a busy
/// consumer.
///
/// Pushing is non-blocking: at capacity the oldest entry is evicted, so the
/// producer (the render loop, or the capture worker) never waits on DDS.
struct Mailbox<T> {
    inner: Mutex<MailboxInner<T>>,
    ready: Condvar,
    capacity: usize,
}

struct MailboxInner<T> {
    queue: VecDeque<T>,
    closed: bool,
}

impl<T> Mailbox<T> {
    fn new(capacity: usize) -> Self {
        Self {
            inner: Mutex::new(MailboxInner {
                queue: VecDeque::new(),
                closed: false,
            }),
            ready: Condvar::new(),
            capacity: capacity.max(1),
        }
    }

    fn push(&self, message: T) {
        let mut inner = self.inner.lock().unwrap();
        if inner.closed {
            return;
        }
        while inner.queue.len() >= self.capacity {
            inner.queue.pop_front();
        }
        inner.queue.push_back(message);
        self.ready.notify_one();
    }

    /// Blocks until a message arrives or the wait times out; `None` means "call
    /// again" (either nothing to send yet, or the mailbox was closed).
    fn pop_timeout(&self, timeout: Duration) -> Option<T> {
        let mut inner = self.inner.lock().unwrap();
        if inner.queue.is_empty() && !inner.closed {
            let (guard, _) = self.ready.wait_timeout(inner, timeout).unwrap();
            inner = guard;
        }
        if inner.closed {
            return None;
        }
        inner.queue.pop_front()
    }

    fn close(&self) {
        let mut inner = self.inner.lock().unwrap();
        inner.closed = true;
        inner.queue.clear();
        self.ready.notify_all();
    }
}

#[derive(Resource)]
pub struct TopicPublisher<T: RosTopic> {
    mailbox: Arc<Mailbox<T::T>>,
}

impl<T: RosTopic> Clone for TopicPublisher<T> {
    fn clone(&self) -> Self {
        Self {
            mailbox: self.mailbox.clone(),
        }
    }
}

impl<T: RosTopic> TopicPublisher<T> {
    fn new(mailbox: Arc<Mailbox<T::T>>) -> Self {
        TopicPublisher { mailbox }
    }

    /// Hand a message to the publisher thread. Never blocks; when the thread is
    /// behind, the oldest queued message is dropped so latency stays bounded.
    pub fn publish(&self, message: T::T) {
        self.mailbox.push(message);
    }
}

#[derive(Resource)]
pub struct TopicSubscriber<T: RosTopic> {
    receiver: Arc<Mutex<Option<T::T>>>,
}

impl<T: RosTopic> TopicSubscriber<T> {
    pub(super) fn new() -> Self {
        TopicSubscriber {
            receiver: Arc::new(Mutex::new(None)),
        }
    }

    /// Takes the most recent message; older ones are already gone.
    pub fn try_recv(&self) -> Option<T::T> {
        self.receiver.lock().unwrap().take()
    }
}

fn subscriber<T: RosTopic>(node: &mut Node, signal: Arc<AtomicBool>) -> TopicSubscriber<T> {
    let mut subscriber = node.subscribe::<T::T>(T::TOPIC, T::QOS).unwrap();
    let sub = TopicSubscriber::new();
    let mutex = sub.receiver.clone();
    std::thread::spawn(move || {
        while !signal.load(std::sync::atomic::Ordering::Acquire) {
            match block_on(futures_lite::StreamExt::next(&mut subscriber)) {
                Some(msg) => {
                    mutex.lock().unwrap().replace(msg);
                }
                None => continue,
            }
        }
    });
    sub
}

/// Own thread per publisher, not a task on the async compute pool.
///
/// `publish` serialises the whole message into CDR and hands it to DDS, which is
/// synchronous CPU work. Doing it on `AsyncComputeTaskPool` made the image
/// publisher compete with the capture readback/conversion tasks for the same
/// workers, so a 4.66 MB frame could hold up the next frame's conversion. A
/// dedicated thread per topic keeps that traffic off the pool, and there are
/// only a handful of topics.
fn publisher<T: RosTopic>(node: &mut Node, signal: Arc<AtomicBool>) -> TopicPublisher<T> {
    let mailbox = Arc::new(Mailbox::new(T::QUEUE));
    let publisher = node.create_publisher(T::TOPIC, T::QOS).unwrap();

    let consumer = mailbox.clone();
    std::thread::spawn(move || {
        while !signal.load(std::sync::atomic::Ordering::Acquire) {
            match consumer.pop_timeout(Duration::from_millis(50)) {
                Some(message) => {
                    let _ = publisher.publish(&message);
                }
                None => continue,
            }
        }
        consumer.close();
    });

    TopicPublisher::new(mailbox)
}

#[macro_export]
macro_rules! subscriber {
    ($signal:expr, $app:ident, $node:ident, $($topic:ty),* $(,)?) => {
        $(
            $app.insert_resource($crate::ros2::topic::subscriber::<$topic>($node, $signal));
        )*
    };
}

pub trait RosTopic {
    type T: WrappedTypesupport + Send + 'static;
    const TOPIC: &'static str;
    const QOS: QosProfile;
    /// How many of this topic's messages may wait for DDS. The mailbox drops the
    /// oldest on overflow, so a small number is the right answer for anything
    /// that is superseded message-by-message.
    const QUEUE: usize = 8;
}

macro_rules! topic {
    ($topic:ident, $msg_typ:ty, $url:literal, $qos:expr, $queue:expr) => {
        #[derive(Clone)]
        pub struct $topic;
        impl RosTopic for $topic {
            type T = $msg_typ;
            const TOPIC: &'static str = $url;
            const QOS: QosProfile = $qos;
            const QUEUE: usize = $queue;
        }
    };
    ($topic:ident, $msg_typ:ty, $url:literal, $qos:expr) => {
        topic!($topic, $msg_typ, $url, $qos, 8);
    };
    ($topic:ident, $msg_typ:ty, $url:literal) => {
        topic!($topic, $msg_typ, $url, ::r2r::QosProfile::default());
    };
    (pub {$($url:literal as $msg_typ:ty as $topic:ident $(queue $queue:tt)? with $qos:expr;)*} $($remaining:tt)*) => {
        $(
            topic!($topic, $msg_typ, $url, $qos $(, $queue)?);
        )*

        pub fn register_pub(atomic:Arc<AtomicBool>, app:&mut App, node:&mut Node) {
            $(
                app.insert_resource(publisher::<$topic>(node, atomic.clone()));
            )*
        }
        topic!($($remaining)*);
    };
    (sub {$($url:literal as $msg_typ:ty as $topic:ident $(queue $queue:tt)? $(with $qos: expr)?;)*} $($remaining:tt)*) => {
        $(
            topic!($topic, $msg_typ, $url $(, $qos)?);
        )*

        pub fn register_sub(atomic:Arc<AtomicBool>, app:&mut App, node:&mut Node) {
            $crate::subscriber!(atomic, app, node, $($topic,)*);
        }
        topic!($($remaining)*);
    };
    ( )=>{}
}

topic!(
    pub {
        // QoS IS TUNED PER TOPIC, AND MESSAGE SIZE DECIDES IT.
        //
        // Best-effort is the right contract for a real-time sensor stream: no
        // retransmission of frames that are already obsolete. But Fast DDS only
        // keeps messages up to its shared-memory segment size (~512 KB by
        // default) on the SHM transport; anything larger falls back to
        // fragmented UDP, where best-effort means the reader has to receive
        // *every* fragment or drop the whole message. Measured: a best-effort
        // `/image_raw` (4.66 MB/frame) delivered exactly zero messages to a
        // subscriber, while the same topic over RELIABLE delivered normally.
        //
        // So: best-effort only for topics that stay small (intrinsics, compressed
        // frames), reliable for the ones that do not.
        "/camera_info" as CameraInfo as CameraInfoTopic queue 4 with QosProfile::sensor_data();
        // 4.66 MB/frame: over the SHM threshold, so it must stay RELIABLE.
        "/image_raw" as Image as ImageRawTopic queue 1 with QosProfile::default();
        // image_transport's naming, so the topic matches a real camera and
        // rm_auto_record's existing config. A queued JPEG is already stale.
        // ~90 KB at the default quality, i.e. comfortably inside SHM.
        "/image_raw/compressed" as CompressedImage as ImageCompressedTopic queue 1 with QosProfile::sensor_data();
        // ~720 KB per cloud at 10 Hz, over the SHM threshold like /image_raw.
        "/livox/lidar" as PointCloud2 as LivoxPointCloudTopic queue 2 with QosProfile::default();
        "/tf" as TFMessage as GlobalTransformTopic queue 16 with QosProfile::default();
        "/simulator/marker" as Marker as OutpostMarkerTopic with QosProfile::default();
        "/gimbal_pose" as PoseStamped as GimbalPoseTopic with QosProfile::default();
        "/odom_pose" as PoseStamped as OdomPoseTopic with QosProfile::default();
        "/muzzle_pose" as PoseStamped as MuzzlePoseTopic with QosProfile::default();
        "/camera_pose" as PoseStamped as CameraPoseTopic with QosProfile::default();
        "/simulator/tech_core/state" as RosString as TechCoreStateTopic with QosProfile::default();
    }
    sub {
        "/rm_gimbal/cmd" as GimbalCmd as GimbalCmdTopic with QosProfile::sensor_data();
    }
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mailbox_keeps_only_the_newest_when_full() {
        let mailbox: Mailbox<u32> = Mailbox::new(2);

        mailbox.push(1);
        mailbox.push(2);
        mailbox.push(3);

        // Overflow drops the OLDEST, so the newest is always retained; delivery
        // itself stays FIFO, so ordering is never inverted.
        assert_eq!(mailbox.pop_timeout(Duration::ZERO), Some(2));
        assert_eq!(mailbox.pop_timeout(Duration::ZERO), Some(3));
        assert_eq!(
            mailbox.pop_timeout(Duration::ZERO),
            None,
            "1 was evicted to make room for 3"
        );
    }

    #[test]
    fn mailbox_of_one_is_strictly_latest_wins() {
        let mailbox: Mailbox<u32> = Mailbox::new(1);

        mailbox.push(1);
        mailbox.push(2);

        assert_eq!(mailbox.pop_timeout(Duration::ZERO), Some(2));
        assert_eq!(mailbox.pop_timeout(Duration::ZERO), None);
    }

    #[test]
    fn mailbox_capacity_is_at_least_one() {
        let mailbox: Mailbox<u32> = Mailbox::new(0);

        mailbox.push(7);
        mailbox.push(8);

        assert_eq!(mailbox.pop_timeout(Duration::ZERO), Some(8));
        assert_eq!(mailbox.pop_timeout(Duration::ZERO), None);
    }

    #[test]
    fn mailbox_pop_waits_for_a_later_push() {
        let mailbox = Arc::new(Mailbox::new(4));
        let producer = mailbox.clone();

        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(20));
            producer.push(42);
        });

        assert_eq!(mailbox.pop_timeout(Duration::from_secs(2)), Some(42));
        handle.join().unwrap();
    }

    #[test]
    fn mailbox_close_releases_the_consumer() {
        let mailbox: Mailbox<u32> = Mailbox::new(4);

        mailbox.push(1);
        mailbox.close();

        assert_eq!(mailbox.pop_timeout(Duration::from_millis(10)), None);
        mailbox.push(2);
        assert_eq!(
            mailbox.pop_timeout(Duration::from_millis(10)),
            None,
            "a closed mailbox ignores further pushes"
        );
    }

    #[test]
    fn image_queues_are_shallow_enough_to_stay_fresh() {
        // One queued image is already one frame of extra latency; anything
        // deeper trades latency for frames the detector can no longer use.
        assert_eq!(ImageRawTopic::QUEUE, 1);
        assert_eq!(ImageCompressedTopic::QUEUE, 1);
        assert_eq!(CameraInfoTopic::QUEUE, 4);
        // tf2 interpolates, but a long backlog of transforms would still push
        // `lookup_transform` into extrapolation failures.
        assert!(GlobalTransformTopic::QUEUE >= 8);
    }

    #[test]
    fn compressed_topic_follows_image_transport_naming() {
        assert_eq!(ImageCompressedTopic::TOPIC, "/image_raw/compressed");
    }
}
