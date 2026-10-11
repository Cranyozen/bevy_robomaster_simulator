//! Turning a captured `rgb8` frame into something affordable to send over DDS.
//!
//! The auto-aim stack only needs the light bars and the armor numbers, but the
//! capture camera renders 1440x1080 — 4.66 MB per frame as raw `rgb8`. At the
//! ~50 fps the render loop reaches, publishing that as `sensor_msgs/Image` put
//! ~230 MB/s through DDS and was the single biggest reason the control loop ran
//! behind the scene. Encoding to JPEG (and optionally downscaling first) shrinks
//! that by more than an order of magnitude at a fidelity the detector cannot
//! tell apart; `/camera_info` is scaled to match, so PnP stays exact.
//!
//! THE ENCODER, NOT THE WIRE, IS THE LIMIT. `image`'s JPEG encoder is scalar
//! (measured ~21 Mpix/s on this machine, 73 ms for one synthetic 1440x1080
//! frame, and worse on real scene detail) — about 5 fps if left at the capture
//! resolution. So `[capture.color] scale` is not a bandwidth knob, it is the
//! knob that decides how many images per second the loop actually gets; see the
//! `scale` docs in `config.toml` for the numbers.

use image::codecs::jpeg::JpegEncoder;
use image::{ImageBuffer, Rgb};
use r2r::sensor_msgs::msg::CompressedImage;
use r2r::std_msgs::msg::Header;
use std::io::Cursor;

/// JPEG-encode an interleaved `rgb8` buffer at `quality` (clamped to `1..=100`).
pub fn encode_jpeg(width: u32, height: u32, data: &[u8], quality: u8) -> Vec<u8> {
    let buffer: ImageBuffer<Rgb<u8>, _> =
        ImageBuffer::from_raw(width, height, data).expect("rgb8 buffer size mismatch");

    let mut cursor = Cursor::new(Vec::new());
    let mut encoder = JpegEncoder::new_with_quality(&mut cursor, quality.clamp(1, 100));
    encoder.encode_image(&buffer).expect("JPEG encode failed");
    cursor.into_inner()
}

pub fn compress_image(
    header: Header,
    width: u32,
    height: u32,
    data: &[u8],
    quality: u8,
) -> CompressedImage {
    CompressedImage {
        header,
        format: "jpeg".to_string(),
        data: encode_jpeg(width, height, data, quality),
    }
}

/// Resize an interleaved `rgb8` buffer to exactly `target_width` x
/// `target_height`, or `None` when that is already its size (in which case the
/// caller should keep using the original slice instead of paying for a copy).
///
/// The target is supplied by the caller — `config::CaptureStreamConfig::
/// published_size()` is the single place that decides it — so the pixels and the
/// `/camera_info` intrinsics can never drift apart.
pub fn rgb_resized(
    width: u32,
    height: u32,
    data: &[u8],
    target_width: u32,
    target_height: u32,
) -> Option<(u32, u32, Vec<u8>)> {
    if width == 0 || height == 0 || data.len() < (width as usize * height as usize * 3) {
        return None;
    }

    let target_width = target_width.clamp(1, width);
    let target_height = target_height.clamp(1, height);
    if (target_width, target_height) == (width, height) {
        return None;
    }

    let resized = box_downscale(data, width, height, target_width, target_height);
    Some((target_width, target_height, resized))
}

/// Integer box-filter downscale: every output pixel is the average of the
/// source rect that maps onto it.
///
/// WHY NOT `image::imageops::resize`: measured at 173 ms for one 1440x1080 ->
/// 720x540 frame (against 5 ms here), because it evaluates a resampling kernel
/// per pixel. That is 30x the entire rest of the capture pipeline, and it lands
/// on the frame-rate critical path — the capture worker is single-flight, so the
/// image rate collapses to ~5/s. A plain box average is also the *right* filter
/// for downscaling a noise-free render: it never skips a light bar the way
/// point sampling can.
fn box_downscale(
    data: &[u8],
    width: u32,
    height: u32,
    target_width: u32,
    target_height: u32,
) -> Vec<u8> {
    let mut out = vec![0u8; target_width as usize * target_height as usize * 3];

    for oy in 0..target_height {
        let y0 = (oy as u64 * height as u64 / target_height as u64) as usize;
        let y1 = (((oy as u64 + 1) * height as u64 / target_height as u64) as usize)
            .max(y0 + 1)
            .min(height as usize);

        for ox in 0..target_width {
            let x0 = (ox as u64 * width as u64 / target_width as u64) as usize;
            let x1 = (((ox as u64 + 1) * width as u64 / target_width as u64) as usize)
                .max(x0 + 1)
                .min(width as usize);

            let mut acc = [0u32; 3];
            let mut samples = 0u32;
            for y in y0..y1 {
                let row_start = (y * width as usize + x0) * 3;
                let row = &data[row_start..row_start + (x1 - x0) * 3];
                for pixel in row.chunks_exact(3) {
                    acc[0] += pixel[0] as u32;
                    acc[1] += pixel[1] as u32;
                    acc[2] += pixel[2] as u32;
                }
                samples += (x1 - x0) as u32;
            }

            let o = (oy as usize * target_width as usize + ox as usize) * 3;
            out[o] = (acc[0] / samples) as u8;
            out[o + 1] = (acc[1] / samples) as u8;
            out[o + 2] = (acc[2] / samples) as u8;
        }
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::{GenericImageView, ImageFormat, RgbImage};

    /// A frame with a couple of saturated light bars on a dark background — the
    /// kind of content the armor detector actually thresholds, and unlike a flat
    /// image it does not let JPEG win for free.
    fn test_frame(width: u32, height: u32) -> Vec<u8> {
        let mut img = RgbImage::from_pixel(width, height, Rgb([20, 24, 30]));
        for x in width / 4..width / 4 + width / 16 {
            for y in height / 4..height * 3 / 4 {
                img.put_pixel(x, y, Rgb([40, 90, 255]));
            }
        }
        img.into_raw()
    }

    #[test]
    fn jpeg_round_trips_dimensions() {
        let (w, h) = (64, 48);
        let data = test_frame(w, h);
        let bytes = encode_jpeg(w, h, &data, 90);

        assert_eq!(&bytes[..2], &[0xFF, 0xD8], "JPEG SOI marker");

        let decoded = image::load_from_memory_with_format(&bytes, ImageFormat::Jpeg).unwrap();
        assert_eq!(decoded.dimensions(), (w, h));
    }

    #[test]
    fn jpeg_is_much_smaller_than_rgb8() {
        let (w, h) = (640, 480);
        let data = test_frame(w, h);
        let bytes = encode_jpeg(w, h, &data, 80);

        assert!(
            bytes.len() * 8 < data.len(),
            "expected at least 8x compression, got {} vs {} bytes",
            bytes.len(),
            data.len()
        );
    }

    #[test]
    fn lower_quality_is_smaller() {
        let (w, h) = (320, 240);
        let data = test_frame(w, h);

        let cheap = encode_jpeg(w, h, &data, 30).len();
        let fine = encode_jpeg(w, h, &data, 95).len();

        assert!(cheap < fine, "{cheap} should be smaller than {fine}");
    }

    #[test]
    fn quality_is_clamped_instead_of_panicking() {
        let data = test_frame(16, 16);

        assert!(!encode_jpeg(16, 16, &data, 0).is_empty());
        assert!(!encode_jpeg(16, 16, &data, 255).is_empty());
    }

    #[test]
    fn resize_returns_none_when_the_target_is_the_source() {
        let data = test_frame(32, 24);

        assert!(rgb_resized(32, 24, &data, 32, 24).is_none());
    }

    #[test]
    fn resize_scales_pixels_and_reports_the_new_size() {
        let data = test_frame(64, 48);

        let (w, h, scaled) = rgb_resized(64, 48, &data, 32, 24).expect("target differs");
        assert_eq!((w, h), (32, 24));
        assert_eq!(scaled.len(), 32 * 24 * 3);

        // The light bar is a quarter of the way in and still bright blue after
        // the downscale: the detector's thresholds survive a halving.
        let px = |x: u32, y: u32| {
            let i = ((y * w + x) * 3) as usize;
            [scaled[i], scaled[i + 1], scaled[i + 2]]
        };
        assert!(px(8, 12)[2] > 200, "light bar stays saturated");
        assert!(px(31, 23)[2] < 60, "background stays dark");
    }

    #[test]
    fn box_downscale_averages_the_source_block() {
        // A 2x2 source of two colours must average, not pick one sample: this is
        // what keeps a 1-2 px light bar from vanishing under a 2x downscale.
        // Columns alternate 0 / 200, so every 2x2 output block averages to 100.
        // Point sampling would instead return 0 or 200 depending on the phase.
        let mut data = vec![0u8; 4 * 4 * 3];
        for y in 0..4 {
            for x in 0..4 {
                let i = (y * 4 + x) * 3;
                let value = if x % 2 == 0 { 0u8 } else { 200u8 };
                data[i..i + 3].copy_from_slice(&[value, value, value]);
            }
        }

        let (w, h, out) = rgb_resized(4, 4, &data, 2, 2).unwrap();
        assert_eq!((w, h), (2, 2));
        assert_eq!(
            &out[0..3],
            &[100, 100, 100],
            "left block averages 0 and 200"
        );
        assert_eq!(
            &out[3..6],
            &[100, 100, 100],
            "right block averages 0 and 200"
        );
    }

    #[test]
    fn box_downscale_rejects_a_truncated_buffer() {
        assert!(rgb_resized(4, 4, &[0u8; 10], 2, 2).is_none());
        assert!(rgb_resized(0, 0, &[], 1, 1).is_none());
    }

    #[test]
    fn resize_never_yields_a_zero_pixel_image() {
        let data = test_frame(4, 4);

        let (w, h, scaled) = rgb_resized(4, 4, &data, 0, 0).expect("target differs");
        assert_eq!((w, h), (1, 1));
        assert_eq!(scaled.len(), 3);
    }

    #[test]
    fn compressed_image_carries_the_jpeg_format_tag() {
        let data = test_frame(32, 32);
        let msg = compress_image(
            Header {
                stamp: Default::default(),
                frame_id: "camera_optical_frame".to_string(),
            },
            32,
            32,
            &data,
            75,
        );

        assert_eq!(msg.format, "jpeg");
        assert_eq!(msg.header.frame_id, "camera_optical_frame");
        let decoded = image::load_from_memory_with_format(&msg.data, ImageFormat::Jpeg).unwrap();
        assert_eq!(decoded.dimensions(), (32, 32));
    }
}
