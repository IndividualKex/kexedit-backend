use kexedit_core::{
    sim, Curvature, Forces, Frame, Keyframe, Point, Float3,
};

use crate::{DurationType, IterationConfig};

const MAX_ITERATIONS: usize = 1_000_000;

pub fn build(
    anchor: &Point,
    config: &IterationConfig,
    driven: bool,
    steering: bool,
    roll_speed: &[Keyframe],
    pitch_speed: &[Keyframe],
    yaw_speed: &[Keyframe],
    driven_velocity: &[Keyframe],
    heart_offset: &[Keyframe],
    friction: &[Keyframe],
    resistance: &[Keyframe],
    anchor_heart: f32,
    anchor_friction: f32,
    anchor_resistance: f32,
) -> Vec<Point> {
    let mut result = Vec::new();
    result.push(*anchor);

    let mut state = *anchor;
    let mut accumulated_roll = 0.0;

    match config.duration_type {
        DurationType::Time => {
            build_time_section(
                config.duration,
                driven,
                steering,
                roll_speed,
                pitch_speed,
                yaw_speed,
                driven_velocity,
                heart_offset,
                friction,
                resistance,
                anchor_heart,
                anchor_friction,
                anchor_resistance,
                &mut state,
                &mut accumulated_roll,
                &mut result,
            );
        }
        DurationType::Distance => {
            build_distance_section(
                config.duration,
                driven,
                steering,
                anchor.heart_arc,
                roll_speed,
                pitch_speed,
                yaw_speed,
                driven_velocity,
                heart_offset,
                friction,
                resistance,
                anchor_heart,
                anchor_friction,
                anchor_resistance,
                &mut state,
                &mut accumulated_roll,
                &mut result,
            );
        }
    }

    result
}

#[allow(clippy::too_many_arguments)]
fn build_time_section(
    duration: f32,
    driven: bool,
    steering: bool,
    roll_speed: &[Keyframe],
    pitch_speed: &[Keyframe],
    yaw_speed: &[Keyframe],
    driven_velocity: &[Keyframe],
    heart_offset: &[Keyframe],
    friction: &[Keyframe],
    resistance: &[Keyframe],
    anchor_heart: f32,
    anchor_friction: f32,
    anchor_resistance: f32,
    state: &mut Point,
    accumulated_roll: &mut f32,
    result: &mut Vec<Point>,
) {
    let mut prev_heart_offset = anchor_heart;
    let mut prev_friction = anchor_friction;

    let point_count = (sim::HZ * duration).floor() as usize;
    for i in 1..point_count {
        let t = i as f32 / sim::HZ;

        let mut prev = *state;

        if driven {
            let velocity = kexedit_core::evaluate(driven_velocity, t, prev.velocity);
            if velocity < sim::MIN_VELOCITY {
                break;
            }
            prev = prev.with_velocity(velocity, prev_heart_offset, prev_friction, true);
        } else if prev.velocity < sim::MIN_VELOCITY {
            if prev.frame().pitch() < 0.0 {
                prev = prev.with_velocity(sim::MIN_VELOCITY, prev_heart_offset, prev_friction, true);
            } else {
                break;
            }
        }

        let heart_offset_val = kexedit_core::evaluate(heart_offset, t, anchor_heart);
        let friction_val = kexedit_core::evaluate(friction, t, anchor_friction);
        let resistance_val = kexedit_core::evaluate(resistance, t, anchor_resistance);

        let pitch_speed_val = kexedit_core::evaluate(pitch_speed, t, 0.0);
        let yaw_speed_val = kexedit_core::evaluate(yaw_speed, t, 0.0);
        let roll_speed_val = kexedit_core::evaluate(roll_speed, t, 0.0);

        let delta_roll = roll_speed_val / sim::HZ;
        let delta_pitch = pitch_speed_val / sim::HZ;
        let delta_yaw = yaw_speed_val / sim::HZ;

        let curr = step_geometric(
            &prev,
            heart_offset_val,
            friction_val,
            resistance_val,
            delta_roll,
            delta_pitch,
            delta_yaw,
            driven,
            steering,
            roll_speed_val,
            accumulated_roll,
        );

        result.push(curr);
        *state = curr;
        prev_heart_offset = heart_offset_val;
        prev_friction = friction_val;
    }
}

#[allow(clippy::too_many_arguments)]
fn build_distance_section(
    duration: f32,
    driven: bool,
    steering: bool,
    anchor_heart_arc: f32,
    roll_speed: &[Keyframe],
    pitch_speed: &[Keyframe],
    yaw_speed: &[Keyframe],
    driven_velocity: &[Keyframe],
    heart_offset: &[Keyframe],
    friction: &[Keyframe],
    resistance: &[Keyframe],
    anchor_heart: f32,
    anchor_friction: f32,
    anchor_resistance: f32,
    state: &mut Point,
    accumulated_roll: &mut f32,
    result: &mut Vec<Point>,
) {
    let mut prev_heart_offset = anchor_heart;
    let mut prev_friction = anchor_friction;

    let end_length = anchor_heart_arc + duration;
    let mut iterations = 0;

    while state.heart_arc < end_length {
        if iterations >= MAX_ITERATIONS {
            break;
        }
        iterations += 1;

        let prev = *state;
        let d = prev.heart_arc - anchor_heart_arc + prev.velocity / sim::HZ;

        let mut prev = prev;
        if driven {
            let velocity = kexedit_core::evaluate(driven_velocity, d, prev.velocity);
            if velocity < sim::MIN_VELOCITY {
                break;
            }
            prev = prev.with_velocity(velocity, prev_heart_offset, prev_friction, true);
        } else if prev.velocity < sim::MIN_VELOCITY {
            if prev.frame().pitch() < 0.0 {
                prev = prev.with_velocity(sim::MIN_VELOCITY, prev_heart_offset, prev_friction, true);
            } else {
                break;
            }
        }

        let heart_offset_val = kexedit_core::evaluate(heart_offset, d, anchor_heart);
        let friction_val = kexedit_core::evaluate(friction, d, anchor_friction);
        let resistance_val = kexedit_core::evaluate(resistance, d, anchor_resistance);

        let pitch_speed_val = kexedit_core::evaluate(pitch_speed, d, 0.0);
        let yaw_speed_val = kexedit_core::evaluate(yaw_speed, d, 0.0);
        let roll_speed_val = kexedit_core::evaluate(roll_speed, d, 0.0);

        let delta_roll = roll_speed_val * (prev.velocity / sim::HZ);
        let delta_pitch = pitch_speed_val * (prev.velocity / sim::HZ);
        let delta_yaw = yaw_speed_val * (prev.velocity / sim::HZ);

        let curr = step_geometric(
            &prev,
            heart_offset_val,
            friction_val,
            resistance_val,
            delta_roll,
            delta_pitch,
            delta_yaw,
            driven,
            steering,
            roll_speed_val,
            accumulated_roll,
        );

        result.push(curr);
        *state = curr;
        prev_heart_offset = heart_offset_val;
        prev_friction = friction_val;
    }
}

#[allow(clippy::too_many_arguments)]
fn step_geometric(
    prev: &Point,
    heart_offset_val: f32,
    friction_val: f32,
    resistance_val: f32,
    delta_roll: f32,
    delta_pitch: f32,
    delta_yaw: f32,
    driven: bool,
    steering: bool,
    roll_speed_val: f32,
    accumulated_roll: &mut f32,
) -> Point {
    let prev_frame = prev.frame();
    let prev_direction = prev.direction;
    let prev_spine_position = prev.spine_position;

    let (curr_direction, curr_lateral, curr_normal, curr_spine_position) = if steering {
        let mut unrolled_frame = prev_frame;
        if accumulated_roll.abs() > sim::EPSILON {
            unrolled_frame = prev_frame.with_roll(-*accumulated_roll);
        }

        let up = if unrolled_frame.normal.y >= 0.0 {
            Float3::UP
        } else {
            -Float3::UP
        };
        let pitch_axis = up.cross(prev_direction).normalize();
        let rotated = unrolled_frame
            .rotate_around(pitch_axis, delta_pitch)
            .with_yaw(delta_yaw);
        let curr_direction = rotated.direction;
        let lateral_unrolled = rotated.lateral;
        let normal_unrolled = rotated.normal;

        let half_step_distance = prev.velocity / (2.0 * sim::HZ);
        let curr_spine_position = prev_spine_position
            + curr_direction * half_step_distance
            + prev_direction * half_step_distance;

        *accumulated_roll += delta_roll;

        let (curr_lateral, curr_normal) = if accumulated_roll.abs() > sim::EPSILON {
            let rerolled = Frame::new(curr_direction, normal_unrolled, lateral_unrolled)
                .with_roll(*accumulated_roll);
            (rerolled.lateral, rerolled.normal)
        } else {
            (lateral_unrolled, normal_unrolled)
        };

        (curr_direction, curr_lateral, curr_normal, curr_spine_position)
    } else {
        let rotated = prev_frame.with_pitch(delta_pitch).with_yaw(delta_yaw);
        let curr_direction = rotated.direction;
        let mut curr_normal = rotated.normal;

        let half_step_distance = prev.velocity / (2.0 * sim::HZ);
        let prev_heart_pos = prev.heart_position(heart_offset_val);
        let curr_heart_pos_if_spine_static = prev_spine_position + curr_normal * heart_offset_val;

        let curr_spine_position = prev_spine_position
            + curr_direction * half_step_distance
            + prev_direction * half_step_distance
            + (prev_heart_pos - curr_heart_pos_if_spine_static);

        let rolled = rotated.with_roll(delta_roll);
        let curr_lateral = rolled.lateral;
        curr_normal = rolled.normal;

        (curr_direction, curr_lateral, curr_normal, curr_spine_position)
    };

    let heart_advance = ((curr_spine_position + curr_normal * heart_offset_val)
        - prev.heart_position(heart_offset_val)).magnitude();
    let new_heart_arc = prev.heart_arc + heart_advance;
    let spine_advance = (curr_spine_position - prev_spine_position).magnitude();
    let new_spine_arc = prev.spine_arc + spine_advance;

    let (new_energy, new_velocity) = if !driven {
        let center_y = (curr_spine_position + curr_normal * (0.9 * heart_offset_val)).y;
        let friction_distance = new_heart_arc - prev.friction_origin;
        sim::update_energy(
            prev.energy,
            prev.velocity,
            center_y,
            friction_distance,
            friction_val,
            resistance_val,
        )
    } else {
        (prev.energy, prev.velocity)
    };

    let curr_frame = Frame::new(curr_direction, curr_normal, curr_lateral);
    let curvature = Curvature::from_frames(curr_frame, prev_frame);
    let forces = Forces::compute(curvature, curr_frame, new_velocity, spine_advance);

    Point::new(
        curr_spine_position,
        curr_direction,
        curr_normal,
        curr_lateral,
        new_velocity,
        new_energy,
        forces.normal,
        forces.lateral,
        new_heart_arc,
        new_spine_arc,
        spine_advance,
        prev.friction_origin,
        roll_speed_val,
        heart_offset_val,
        friction_val,
        resistance_val,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_time_section_creates_points() {
        let anchor = Point::DEFAULT;
        let config = IterationConfig::new(0.5, DurationType::Time);

        let result = build(
            &anchor,
            &config,
            false,
            false,
            &[],
            &[],
            &[],
            &[],
            &[],
            &[],
            &[],
            1.1,
            0.0,
            0.0,
        );

        assert!(result.len() > 1);
        assert_eq!(result[0], anchor);
    }

    #[test]
    fn build_distance_section_creates_points() {
        let anchor = Point::DEFAULT;
        let config = IterationConfig::new(5.0, DurationType::Distance);

        let result = build(
            &anchor,
            &config,
            false,
            false,
            &[],
            &[],
            &[],
            &[],
            &[],
            &[],
            &[],
            1.1,
            0.0,
            0.0,
        );

        assert!(result.len() > 1);
        assert_eq!(result[0], anchor);
    }

    #[test]
    fn build_with_steering_creates_points() {
        let anchor = Point::DEFAULT;
        let config = IterationConfig::new(0.5, DurationType::Time);

        let result = build(
            &anchor,
            &config,
            false,
            true,
            &[],
            &[],
            &[],
            &[],
            &[],
            &[],
            &[],
            1.1,
            0.0,
            0.0,
        );

        assert!(result.len() > 1);
        assert_eq!(result[0], anchor);
    }

    #[test]
    fn step_geometric_no_steering_updates_position() {
        let anchor = Point::DEFAULT;
        let mut accumulated_roll = 0.0;

        let result = step_geometric(
            &anchor,
            1.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            false,
            false,
            0.0,
            &mut accumulated_roll,
        );

        assert_ne!(result.spine_position, anchor.spine_position);
    }

    #[test]
    fn step_geometric_with_steering_updates_position() {
        let anchor = Point::DEFAULT;
        let mut accumulated_roll = 0.0;

        let result = step_geometric(
            &anchor,
            1.1,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            false,
            true,
            0.1,
            &mut accumulated_roll,
        );

        assert_ne!(result.spine_position, anchor.spine_position);
        assert!(accumulated_roll.abs() > 0.0);
    }

    mod golden_tests {
        use super::*;
        use crate::golden::GoldTrackData;
        use crate::point_comparer::assert_points_match_gold;

        fn build_from_gold_section(section: &crate::golden::GoldSection) -> Vec<Point> {
            let anchor = section.inputs.anchor.to_point();
            let duration = section.inputs.duration.as_ref().unwrap();
            let config = IterationConfig::new(
                duration.value,
                duration.to_duration_type(),
            );
            let keyframes = section.inputs.keyframes.as_ref();
            let overrides = section.inputs.property_overrides.as_ref();

            let roll_speed: Vec<_> = keyframes
                .map(|k| k.roll_speed.iter().map(|kf| kf.to_keyframe()).collect())
                .unwrap_or_default();
            let pitch_speed: Vec<_> = keyframes
                .map(|k| k.pitch_speed.iter().map(|kf| kf.to_keyframe()).collect())
                .unwrap_or_default();
            let yaw_speed: Vec<_> = keyframes
                .map(|k| k.yaw_speed.iter().map(|kf| kf.to_keyframe()).collect())
                .unwrap_or_default();
            let fixed_velocity: Vec<_> = keyframes
                .map(|k| k.fixed_velocity.iter().map(|kf| kf.to_keyframe()).collect())
                .unwrap_or_default();
            let heart_offset: Vec<_> = keyframes
                .map(|k| k.heart.iter().map(|kf| kf.to_keyframe()).collect())
                .unwrap_or_default();
            let friction: Vec<_> = keyframes
                .map(|k| k.friction.iter().map(|kf| kf.to_keyframe()).collect())
                .unwrap_or_default();
            let resistance: Vec<_> = keyframes
                .map(|k| k.resistance.iter().map(|kf| kf.to_keyframe()).collect())
                .unwrap_or_default();

            build(
                &anchor,
                &config,
                overrides.map(|o| o.fixed_velocity).unwrap_or(false),
                section.inputs.steering,
                &roll_speed,
                &pitch_speed,
                &yaw_speed,
                &fixed_velocity,
                &heart_offset,
                &friction,
                &resistance,
                section.inputs.anchor.heart,
                section.inputs.anchor.friction,
                section.inputs.anchor.resistance,
            )
        }

        #[test]
        fn shuttle_geometric_section1_matches_gold() {
            let data = GoldTrackData::load("../../Assets/Tests/TrackData/shuttle.json")
                .expect("Failed to load shuttle.json");
            let sections = data.get_geometric_sections();
            assert!(!sections.is_empty(), "No geometric sections found in shuttle.json");

            let section = sections[0];
            let result = build_from_gold_section(section);

            assert_points_match_gold(&result, &section.outputs.points);
        }

        #[test]
        fn shuttle_geometric_section2_matches_gold() {
            let data = GoldTrackData::load("../../Assets/Tests/TrackData/shuttle.json")
                .expect("Failed to load shuttle.json");
            let sections = data.get_geometric_sections();
            assert!(sections.len() >= 2, "Expected at least 2 geometric sections in shuttle.json");

            let section = sections[1];
            let result = build_from_gold_section(section);

            assert_points_match_gold(&result, &section.outputs.points);
        }

        #[test]
        fn veloci_geometric_section1_matches_gold() {
            let data = GoldTrackData::load("../../Assets/Tests/TrackData/veloci.json")
                .expect("Failed to load veloci.json");
            let sections = data.get_geometric_sections();
            assert!(sections.len() >= 1, "Expected at least 1 geometric section in veloci.json");

            let section = sections[0];
            let result = build_from_gold_section(section);

            assert_points_match_gold(&result, &section.outputs.points);
        }

        #[test]
        fn veloci_geometric_section2_matches_gold() {
            let data = GoldTrackData::load("../../Assets/Tests/TrackData/veloci.json")
                .expect("Failed to load veloci.json");
            let sections = data.get_geometric_sections();
            assert!(sections.len() >= 2, "Expected at least 2 geometric sections in veloci.json");

            let section = sections[1];
            let result = build_from_gold_section(section);

            assert_points_match_gold(&result, &section.outputs.points);
        }

        #[test]
        fn veloci_geometric_section3_matches_gold() {
            let data = GoldTrackData::load("../../Assets/Tests/TrackData/veloci.json")
                .expect("Failed to load veloci.json");
            let sections = data.get_geometric_sections();
            assert!(sections.len() >= 3, "Expected at least 3 geometric sections in veloci.json");

            let section = sections[2];
            let result = build_from_gold_section(section);

            assert_points_match_gold(&result, &section.outputs.points);
        }

        #[test]
        fn veloci_geometric_section4_matches_gold() {
            let data = GoldTrackData::load("../../Assets/Tests/TrackData/veloci.json")
                .expect("Failed to load veloci.json");
            let sections = data.get_geometric_sections();
            assert!(sections.len() >= 4, "Expected at least 4 geometric sections in veloci.json");

            let section = sections[3];
            let result = build_from_gold_section(section);

            assert_points_match_gold(&result, &section.outputs.points);
        }
    }
}
