use kexedit_core::{sim, Curvature, Forces, Frame, Keyframe, PhysicsParams, Point};

use crate::{DurationType, IterationConfig};

const MAX_ITERATIONS: usize = 1_000_000;

fn step_by_forces(
    prev: &Frame,
    normal_force: f32,
    lateral_force: f32,
    velocity: f32,
    heart_advance: f32,
) -> Frame {
    let force_vec = prev.normal * (-normal_force)
        + prev.lateral * (-lateral_force)
        + kexedit_core::Float3::new(0.0, -1.0, 0.0);
    let normal_accel = -force_vec.dot(prev.normal) * sim::G;
    let lateral_accel = -force_vec.dot(prev.lateral) * sim::G;

    let estimated_velocity = if heart_advance.abs() < sim::EPSILON {
        velocity
    } else {
        heart_advance * sim::HZ
    };
    let estimated_velocity = if estimated_velocity.abs() < sim::EPSILON {
        sim::EPSILON
    } else {
        estimated_velocity
    };
    let safe_velocity = if velocity.abs() < sim::EPSILON {
        sim::EPSILON
    } else {
        velocity
    };

    let q_normal = kexedit_core::Quaternion::from_axis_angle(
        prev.lateral,
        normal_accel / estimated_velocity / sim::HZ,
    );
    let q_lateral = kexedit_core::Quaternion::from_axis_angle(
        prev.normal,
        -lateral_accel / safe_velocity / sim::HZ,
    );
    let combined = q_normal * q_lateral;

    let new_direction = combined.mul_vec(prev.direction).normalize();
    let new_lateral = q_lateral.mul_vec(prev.lateral).normalize();
    let new_normal = new_direction.cross(new_lateral).normalize();

    Frame::new(new_direction, new_normal, new_lateral)
}

pub fn advance(
    prev: &Point,
    target_normal_force: f32,
    target_lateral_force: f32,
    physics: &PhysicsParams,
    roll_speed_val: f32,
) -> Point {
    let prev_frame = prev.frame();
    let new_frame = step_by_forces(
        &prev_frame,
        target_normal_force,
        target_lateral_force,
        prev.velocity,
        prev.heart_advance,
    );

    let curr_direction = new_frame.direction;
    let mut curr_normal = new_frame.normal;

    let half_step_distance = prev.velocity / (2.0 * sim::HZ);
    let prev_spine_pos = prev.spine_position(physics.heart_offset);
    let curr_spine_pos_if_heart_static = prev.heart_position + curr_normal * physics.heart_offset;

    let displacement = curr_direction * half_step_distance
        + prev.direction * half_step_distance
        + (prev_spine_pos - curr_spine_pos_if_heart_static);
    let curr_heart_position = prev.heart_position + displacement;

    let rolled_frame = new_frame.with_roll(physics.delta_roll);
    let curr_lateral = rolled_frame.lateral;
    curr_normal = rolled_frame.normal;

    let spine_advance = ((curr_heart_position + curr_normal * physics.heart_offset)
        - prev.spine_position(physics.heart_offset))
    .magnitude();
    let new_spine_arc = prev.spine_arc + spine_advance;
    let heart_advance = (curr_heart_position - prev.heart_position).magnitude();
    let new_heart_arc = prev.heart_arc + heart_advance;

    let (new_energy, new_velocity) = if !physics.driven {
        let center_y = (curr_heart_position + curr_normal * (0.9 * physics.heart_offset)).y;
        let friction_distance = new_spine_arc - prev.friction_origin;
        sim::update_energy(
            prev.energy,
            prev.velocity,
            center_y,
            friction_distance,
            physics.friction,
            physics.resistance,
        )
    } else {
        (prev.energy, prev.velocity)
    };

    let curr_frame = Frame::new(curr_direction, curr_normal, curr_lateral);
    let curvature = Curvature::from_frames(curr_frame, prev_frame);
    let forces = Forces::compute(curvature, curr_frame, new_velocity, spine_advance);

    Point::new(
        curr_heart_position,
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
        physics.heart_offset,
        physics.friction,
        physics.resistance,
    )
}

pub fn build(
    anchor: &Point,
    config: &IterationConfig,
    driven: bool,
    roll_speed: &[Keyframe],
    normal_force: &[Keyframe],
    lateral_force: &[Keyframe],
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

    match config.duration_type {
        DurationType::Time => {
            build_time_section(
                config.duration,
                driven,
                roll_speed,
                normal_force,
                lateral_force,
                driven_velocity,
                heart_offset,
                friction,
                resistance,
                anchor_heart,
                anchor_friction,
                anchor_resistance,
                &mut state,
                &mut result,
            );
        }
        DurationType::Distance => {
            build_distance_section(
                config.duration,
                driven,
                anchor.spine_arc,
                roll_speed,
                normal_force,
                lateral_force,
                driven_velocity,
                heart_offset,
                friction,
                resistance,
                anchor_heart,
                anchor_friction,
                anchor_resistance,
                &mut state,
                &mut result,
            );
        }
    }

    result
}

fn build_time_section(
    duration: f32,
    driven: bool,
    roll_speed: &[Keyframe],
    normal_force: &[Keyframe],
    lateral_force: &[Keyframe],
    driven_velocity: &[Keyframe],
    heart_offset: &[Keyframe],
    friction: &[Keyframe],
    resistance: &[Keyframe],
    anchor_heart: f32,
    anchor_friction: f32,
    anchor_resistance: f32,
    state: &mut Point,
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
                prev =
                    prev.with_velocity(sim::MIN_VELOCITY, prev_heart_offset, prev_friction, true);
            } else {
                break;
            }
        }

        let heart_offset_val = kexedit_core::evaluate(heart_offset, t, anchor_heart);
        let friction_val = kexedit_core::evaluate(friction, t, anchor_friction);
        let resistance_val = kexedit_core::evaluate(resistance, t, anchor_resistance);

        let target_normal_force = kexedit_core::evaluate(normal_force, t, 1.0);
        let target_lateral_force = kexedit_core::evaluate(lateral_force, t, 0.0);
        let roll_speed_val = kexedit_core::evaluate(roll_speed, t, 0.0);
        let delta_roll = roll_speed_val / sim::HZ;

        let physics = PhysicsParams::new(
            heart_offset_val,
            friction_val,
            resistance_val,
            delta_roll,
            driven,
        );
        let curr = advance(
            &prev,
            target_normal_force,
            target_lateral_force,
            &physics,
            roll_speed_val,
        );

        result.push(curr);
        *state = curr;
        prev_heart_offset = heart_offset_val;
        prev_friction = friction_val;
    }
}

fn build_distance_section(
    duration: f32,
    driven: bool,
    anchor_spine_arc: f32,
    roll_speed: &[Keyframe],
    normal_force: &[Keyframe],
    lateral_force: &[Keyframe],
    driven_velocity: &[Keyframe],
    heart_offset: &[Keyframe],
    friction: &[Keyframe],
    resistance: &[Keyframe],
    anchor_heart: f32,
    anchor_friction: f32,
    anchor_resistance: f32,
    state: &mut Point,
    result: &mut Vec<Point>,
) {
    let mut prev_heart_offset = anchor_heart;
    let mut prev_friction = anchor_friction;

    let end_length = anchor_spine_arc + duration;
    let mut iterations = 0;

    while state.spine_arc < end_length {
        if iterations >= MAX_ITERATIONS {
            break;
        }
        iterations += 1;

        let prev = *state;
        let d = prev.spine_arc - anchor_spine_arc + prev.velocity / sim::HZ;

        let mut prev = prev;
        if driven {
            let velocity = kexedit_core::evaluate(driven_velocity, d, prev.velocity);
            if velocity < sim::MIN_VELOCITY {
                break;
            }
            prev = prev.with_velocity(velocity, prev_heart_offset, prev_friction, true);
        } else if prev.velocity < sim::MIN_VELOCITY {
            if prev.frame().pitch() < 0.0 {
                prev =
                    prev.with_velocity(sim::MIN_VELOCITY, prev_heart_offset, prev_friction, true);
            } else {
                break;
            }
        }

        let heart_offset_val = kexedit_core::evaluate(heart_offset, d, anchor_heart);
        let friction_val = kexedit_core::evaluate(friction, d, anchor_friction);
        let resistance_val = kexedit_core::evaluate(resistance, d, anchor_resistance);

        let target_normal_force = kexedit_core::evaluate(normal_force, d, 1.0);
        let target_lateral_force = kexedit_core::evaluate(lateral_force, d, 0.0);
        let roll_speed_val = kexedit_core::evaluate(roll_speed, d, 0.0);
        let delta_roll = roll_speed_val * (prev.velocity / sim::HZ);

        let physics = PhysicsParams::new(
            heart_offset_val,
            friction_val,
            resistance_val,
            delta_roll,
            driven,
        );
        let curr = advance(
            &prev,
            target_normal_force,
            target_lateral_force,
            &physics,
            roll_speed_val,
        );

        result.push(curr);
        *state = curr;
        prev_heart_offset = heart_offset_val;
        prev_friction = friction_val;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kexedit_core::Float3;

    #[test]
    fn advance_updates_position() {
        let anchor = Point::DEFAULT;
        let physics = PhysicsParams::new(1.1, 0.0, 0.0, 0.0, false);

        let result = advance(&anchor, 1.0, 0.0, &physics, 0.0);

        assert_ne!(result.heart_position, anchor.heart_position);
        assert!(result.heart_advance > 0.0);
    }

    #[test]
    fn advance_updates_velocity_when_not_driven() {
        let anchor = Point::create(
            Float3::new(0.0, 10.0, 0.0),
            Float3::BACK,
            0.0,
            10.0,
            1.1,
            0.0,
            0.0,
        );
        let physics = PhysicsParams::new(1.1, 0.0, 0.0, 0.0, false);

        let result = advance(&anchor, 1.0, 0.0, &physics, 0.0);

        assert!(result.velocity > 0.0);
    }

    #[test]
    fn build_time_section_creates_points() {
        let anchor = Point::DEFAULT;
        let config = IterationConfig::new(0.5, DurationType::Time);

        let result = build(
            &anchor,
            &config,
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
    fn build_respects_min_velocity_threshold() {
        let slow_anchor = Point::create(
            Float3::new(0.0, 3.0, 0.0),
            Float3::BACK,
            0.0,
            0.001,
            1.1,
            0.0,
            0.0,
        );
        let config = IterationConfig::new(1.0, DurationType::Time);

        let result = build(
            &slow_anchor,
            &config,
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

        assert!(result.len() <= (sim::HZ * config.duration).round() as usize);
    }

    mod golden_tests {
        use super::*;
        use crate::golden::GoldTrackData;
        use crate::point_comparer::assert_points_match_gold;

        fn build_from_gold_section(section: &crate::golden::GoldSection) -> Vec<Point> {
            let anchor = section.inputs.anchor.to_point();
            let duration = section.inputs.duration.as_ref().unwrap();
            let config = IterationConfig::new(duration.value, duration.to_duration_type());
            let keyframes = section.inputs.keyframes.as_ref();
            let overrides = section.inputs.property_overrides.as_ref();

            let roll_speed: Vec<_> = keyframes
                .map(|k| k.roll_speed.iter().map(|kf| kf.to_keyframe()).collect())
                .unwrap_or_default();
            let normal_force: Vec<_> = keyframes
                .map(|k| k.normal_force.iter().map(|kf| kf.to_keyframe()).collect())
                .unwrap_or_default();
            let lateral_force: Vec<_> = keyframes
                .map(|k| k.lateral_force.iter().map(|kf| kf.to_keyframe()).collect())
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
                &roll_speed,
                &normal_force,
                &lateral_force,
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
        fn shuttle_force_section_matches_gold() {
            let data = GoldTrackData::load("../test-data/shuttle.json")
                .expect("Failed to load shuttle.json");
            let sections = data.get_force_sections();
            assert!(
                !sections.is_empty(),
                "No force sections found in shuttle.json"
            );

            let section = sections[0];
            let result = build_from_gold_section(section);

            assert_points_match_gold(&result, &section.outputs.points);
        }

        #[test]
        fn veloci_force_section1_matches_gold() {
            let data = GoldTrackData::load("../test-data/veloci.json")
                .expect("Failed to load veloci.json");
            let sections = data.get_force_sections();
            assert!(
                sections.len() >= 1,
                "Expected at least 1 force section in veloci.json"
            );

            let section = sections[0];
            let result = build_from_gold_section(section);

            assert_points_match_gold(&result, &section.outputs.points);
        }

        #[test]
        fn veloci_force_section2_matches_gold() {
            let data = GoldTrackData::load("../test-data/veloci.json")
                .expect("Failed to load veloci.json");
            let sections = data.get_force_sections();
            assert!(
                sections.len() >= 2,
                "Expected at least 2 force sections in veloci.json"
            );

            let section = sections[1];
            let result = build_from_gold_section(section);

            assert_points_match_gold(&result, &section.outputs.points);
        }
    }
}
