use kexedit_core::{Frame, Float3, Point, Keyframe, Quaternion};
use kexedit_nodes::{DurationType, IterationConfig};

#[no_mangle]
pub unsafe extern "C" fn kexedit_frame_rotate_around(
    frame: *const Frame,
    axis: *const Float3,
    angle: f32,
    out: *mut Frame,
) {
    if frame.is_null() || axis.is_null() || out.is_null() {
        return;
    }

    let frame_val = *frame;
    let axis_val = *axis;
    let result = frame_val.rotate_around(axis_val, angle);

    *out = result;
}

#[no_mangle]
pub unsafe extern "C" fn kexedit_frame_with_roll(
    frame: *const Frame,
    delta_roll: f32,
    out: *mut Frame,
) {
    if frame.is_null() || out.is_null() {
        return;
    }

    let frame_val = *frame;
    let result = frame_val.with_roll(delta_roll);

    *out = result;
}

#[no_mangle]
pub unsafe extern "C" fn kexedit_frame_with_pitch(
    frame: *const Frame,
    delta_pitch: f32,
    out: *mut Frame,
) {
    if frame.is_null() || out.is_null() {
        return;
    }

    let frame_val = *frame;
    let result = frame_val.with_pitch(delta_pitch);

    *out = result;
}

#[no_mangle]
pub unsafe extern "C" fn kexedit_frame_with_yaw(
    frame: *const Frame,
    delta_yaw: f32,
    out: *mut Frame,
) {
    if frame.is_null() || out.is_null() {
        return;
    }

    let frame_val = *frame;
    let result = frame_val.with_yaw(delta_yaw);

    *out = result;
}

#[no_mangle]
pub unsafe extern "C" fn kexedit_frame_roll(frame: *const Frame) -> f32 {
    if frame.is_null() {
        return 0.0;
    }

    let frame_val = *frame;
    frame_val.roll()
}

#[no_mangle]
pub unsafe extern "C" fn kexedit_force_build(
    anchor: *const Point,
    duration: f32,
    duration_type: i32,
    driven: bool,
    roll_speed: *const Keyframe,
    roll_speed_len: usize,
    normal_force: *const Keyframe,
    normal_force_len: usize,
    lateral_force: *const Keyframe,
    lateral_force_len: usize,
    driven_velocity: *const Keyframe,
    driven_velocity_len: usize,
    heart_offset: *const Keyframe,
    heart_offset_len: usize,
    friction: *const Keyframe,
    friction_len: usize,
    resistance: *const Keyframe,
    resistance_len: usize,
    anchor_heart: f32,
    anchor_friction: f32,
    anchor_resistance: f32,
    out_points: *mut Point,
    out_len: *mut usize,
    max_len: usize,
) -> i32 {
    if anchor.is_null() || out_points.is_null() || out_len.is_null() {
        return -1;
    }

    let anchor_val = *anchor;
    let config = IterationConfig::new(
        duration,
        match duration_type {
            0 => DurationType::Time,
            1 => DurationType::Distance,
            _ => return -2,
        },
    );

    let roll_speed_slice = if roll_speed.is_null() {
        &[]
    } else {
        std::slice::from_raw_parts(roll_speed, roll_speed_len)
    };
    let normal_force_slice = if normal_force.is_null() {
        &[]
    } else {
        std::slice::from_raw_parts(normal_force, normal_force_len)
    };
    let lateral_force_slice = if lateral_force.is_null() {
        &[]
    } else {
        std::slice::from_raw_parts(lateral_force, lateral_force_len)
    };
    let driven_velocity_slice = if driven_velocity.is_null() {
        &[]
    } else {
        std::slice::from_raw_parts(driven_velocity, driven_velocity_len)
    };
    let heart_offset_slice = if heart_offset.is_null() {
        &[]
    } else {
        std::slice::from_raw_parts(heart_offset, heart_offset_len)
    };
    let friction_slice = if friction.is_null() {
        &[]
    } else {
        std::slice::from_raw_parts(friction, friction_len)
    };
    let resistance_slice = if resistance.is_null() {
        &[]
    } else {
        std::slice::from_raw_parts(resistance, resistance_len)
    };

    let result = kexedit_nodes::force::build(
        &anchor_val,
        &config,
        driven,
        roll_speed_slice,
        normal_force_slice,
        lateral_force_slice,
        driven_velocity_slice,
        heart_offset_slice,
        friction_slice,
        resistance_slice,
        anchor_heart,
        anchor_friction,
        anchor_resistance,
    );

    let result_len = result.len();
    if result_len > max_len {
        return -3;
    }

    for (i, point) in result.iter().enumerate() {
        *out_points.add(i) = *point;
    }
    *out_len = result_len;

    0
}

#[no_mangle]
pub unsafe extern "C" fn kexedit_keyframe_evaluate(
    keyframes: *const Keyframe,
    keyframes_len: usize,
    t: f32,
    default_value: f32,
) -> f32 {
    let keyframes_slice = if keyframes.is_null() || keyframes_len == 0 {
        &[]
    } else {
        std::slice::from_raw_parts(keyframes, keyframes_len)
    };
    kexedit_core::evaluate(keyframes_slice, t, default_value)
}

#[no_mangle]
pub extern "C" fn kexedit_quat_mul(a: Quaternion, b: Quaternion) -> Quaternion {
    a.mul(b)
}

#[no_mangle]
pub extern "C" fn kexedit_quat_mul_vec(q: Quaternion, v: Float3) -> Float3 {
    q.mul_vec(v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_ffi_rotate_around() {
        let frame = Frame::DEFAULT;
        let axis = Float3::UP;
        let angle = PI / 2.0;
        let mut out = Frame::DEFAULT;

        unsafe {
            kexedit_frame_rotate_around(&frame, &axis, angle, &mut out);
        }

        assert!((out.direction.x - -1.0).abs() < 1e-6);
        assert!((out.direction.y - 0.0).abs() < 1e-6);
        assert!((out.direction.z - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_ffi_null_safety() {
        let frame = Frame::DEFAULT;
        let axis = Float3::UP;
        let angle = PI / 2.0;
        let mut out = Frame::DEFAULT;

        unsafe {
            kexedit_frame_rotate_around(std::ptr::null(), &axis, angle, &mut out);
            kexedit_frame_rotate_around(&frame, std::ptr::null(), angle, &mut out);
            kexedit_frame_rotate_around(&frame, &axis, angle, std::ptr::null_mut());
        }
    }
}
