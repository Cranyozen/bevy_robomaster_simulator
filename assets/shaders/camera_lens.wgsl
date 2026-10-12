// Simulated camera lens and sensor for the capture camera.
//
// WHY IT LIVES ON THE CAPTURE CAMERA: the layer has to be part of the pixels the
// capture camera produces, not something applied to a copy of them. Then the
// on-screen preview and the ROS 2 / Talos readback cannot disagree, and the
// published image keeps the effect no matter what the preview toggle does.
//
// WHY ONE PASS, NO LOOKUP TEXTURES: every ingredient is a handful of ALU
// instructions, so the whole layer is a single fullscreen triangle with one
// 32-byte uniform. The four effects are the ones that make a clean render look
// like a machine-vision camera rather than a video game: a lens vignette, lateral
// chromatic aberration, radial (barrel/pincushion) distortion, and sensor grain.
// Each is skipped when its strength is zero, so an unused knob costs nothing.

#import bevy_core_pipeline::fullscreen_vertex_shader::FullscreenVertexOutput

// Mirrors `CameraLensEffect` in `src/capture/effect.rs`. Keep the field order in
// sync: the uniform is written with encase's tight packing, so a reordered struct
// silently feeds the shader the wrong numbers.
struct CameraLens {
    vignette: f32,
    vignette_radius: f32,
    vignette_smoothness: f32,
    chromatic_aberration: f32,
    lens_distortion: f32,
    lens_distortion_scale: f32,
    grain: f32,
    seed: f32,
};

@group(0) @binding(0) var screen_texture: texture_2d<f32>;
@group(0) @binding(1) var texture_sampler: sampler;
@group(0) @binding(2) var<uniform> lens: CameraLens;

const CENTER: vec2<f32> = vec2<f32>(0.5, 0.5);

// Radial distortion, the k1 term of the Brown-Conrady model with p1 = p2 = k2 = 0
// (positive = barrel, negative = pincushion). `scale` zooms back in to crop the
// stretched edge pixels a strong `k1` leaves behind.
fn distort(uv: vec2<f32>) -> vec2<f32> {
    let centered = uv - CENTER;
    let r2 = dot(centered, centered);
    let zoom = max(lens.lens_distortion_scale, 1e-3);
    return CENTER + centered * (1.0 + lens.lens_distortion * r2) / zoom;
}

// Lateral chromatic aberration: red and blue are sampled slightly further out
// than green. Green stays exactly where it was so the luminance the armour
// detector thresholds keeps its geometry; only the colour fringes move.
fn chromatic(uv: vec2<f32>) -> vec3<f32> {
    let centered = uv - CENTER;
    let spread = lens.chromatic_aberration;
    let red = textureSampleLevel(screen_texture, texture_sampler, CENTER + centered * (1.0 + spread), 0.0).r;
    let green = textureSampleLevel(screen_texture, texture_sampler, uv, 0.0).g;
    let blue = textureSampleLevel(screen_texture, texture_sampler, CENTER + centered * (1.0 - spread), 0.0).b;
    return vec3<f32>(red, green, blue);
}

// Vignette: `radius` is the normalized distance (0 = centre, 1 = image corner)
// where the darkening starts, `smoothness` is how far it takes to reach full
// strength. Aspect-corrected, so the falloff is elliptical with the frame rather
// than circular in UV space.
fn vignette(uv: vec2<f32>, aspect: f32) -> f32 {
    let centered = (uv - CENTER) * vec2<f32>(aspect, 1.0);
    let corner = length(vec2<f32>(0.5 * aspect, 0.5));
    let r = length(centered) / max(corner, 1e-6);
    let falloff = smoothstep(lens.vignette_radius, lens.vignette_radius + max(lens.vignette_smoothness, 1e-3), r);
    return 1.0 - lens.vignette * falloff;
}

// Hash without `sin`: the classic sin-based hash loses precision differently on
// different GPUs, and this one is stable at any coordinate magnitude.
fn hash21(p: vec2<f32>) -> f32 {
    var p3 = fract(vec3<f32>(p.x, p.y, p.x) * vec3<f32>(0.1031, 0.1030, 0.0973));
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

@fragment
fn fragment(in: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    let size = vec2<f32>(textureDimensions(screen_texture));
    let aspect = size.x / max(size.y, 1.0);

    var uv = in.uv;
    if (abs(lens.lens_distortion) > 0.0) {
        uv = distort(uv);
    }

    var color: vec3<f32>;
    if (lens.chromatic_aberration > 0.0) {
        color = chromatic(uv);
    } else {
        color = textureSampleLevel(screen_texture, texture_sampler, uv, 0.0).rgb;
    }

    if (lens.vignette > 0.0) {
        color *= vignette(in.uv, aspect);
    }

    // Sensor grain: multiplicative gain noise, which is what keeps its amplitude
    // proportional to the signal the way a real sensor's does, and what makes it
    // vanish in true black instead of turning it grey. `seed` advances every
    // frame, so the pattern is a new one each frame rather than a static texture.
    if (lens.grain > 0.0) {
        let noise = hash21(in.uv * size + vec2<f32>(lens.seed, lens.seed * 1.7)) - 0.5;
        color *= 1.0 + noise * 2.0 * lens.grain;
    }

    return vec4<f32>(max(color, vec3<f32>(0.0)), 1.0);
}
