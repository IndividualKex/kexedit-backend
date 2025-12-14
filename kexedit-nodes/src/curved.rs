use kexedit_core::{
    evaluate, sim, Curvature, Forces, Frame, Float3, Keyframe, Point, Quaternion,
};

const MAX_ITERATIONS: usize = 1_000_000;

pub struct CurvedNode;

impl CurvedNode {
    #[allow(clippy::too_many_arguments)]
    pub fn build(
        anchor: &Point,
        radius: f32,
        arc: f32,
        axis: f32,
        lead_in: f32,
        lead_out: f32,
        driven: bool,
        roll_speed: &[Keyframe],
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
        let mut angle = 0.0;
        let lead_out_start_angle = arc - lead_out;
        let mut lead_out_started = false;
        let mut lead_out_start_state = *anchor;
        let mut actual_lead_out = 0.0;

        let mut prev_heart_offset = anchor_heart;
        let mut prev_friction = anchor_friction;

        let mut iterations = 0;
        let mut index = 0;

        while angle < arc - sim::EPSILON {
            iterations += 1;
            if iterations > MAX_ITERATIONS {
                break;
            }

            let mut prev = state;
            let t = index as f32 / sim::HZ;

            if driven {
                let velocity = evaluate(driven_velocity, t, prev.velocity);
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

            let heart_offset_val = evaluate(heart_offset, t, anchor_heart);
            let friction_val = evaluate(friction, t, anchor_friction);
            let resistance_val = evaluate(resistance, t, anchor_resistance);

            let mut delta_angle = prev.velocity / radius / sim::HZ * 1.0f32.to_degrees();

            if lead_in > 0.0 {
                let distance_from_start = prev.heart_arc - anchor.heart_arc;
                let expected_lead_in_distance = 1.997 / sim::HZ * prev.velocity / delta_angle * lead_in;
                let f_trans = distance_from_start / expected_lead_in_distance;
                if f_trans <= 1.0 {
                    let dampening = f_trans * f_trans * (3.0 + f_trans * (-2.0));
                    delta_angle *= dampening;
                }
            }

            if !lead_out_started && angle > lead_out_start_angle {
                lead_out_started = true;
                lead_out_start_state = prev;
                actual_lead_out = arc - angle;
            }

            if lead_out_started && lead_out > 0.0 {
                let distance_from_lead_out_start = prev.heart_arc - lead_out_start_state.heart_arc;
                let expected_lead_out_distance = 1.997 / sim::HZ * prev.velocity / delta_angle * actual_lead_out;
                let f_trans = 1.0 - distance_from_lead_out_start / expected_lead_out_distance;
                if f_trans >= 0.0 {
                    let dampening = f_trans * f_trans * (3.0 + f_trans * (-2.0));
                    delta_angle *= dampening;
                } else {
                    break;
                }
            }

            angle += delta_angle;
            let roll_speed_val = evaluate(roll_speed, angle, 0.0);

            let curr = step_curved(
                &prev,
                axis,
                delta_angle,
                roll_speed_val,
                heart_offset_val,
                friction_val,
                resistance_val,
                driven,
            );

            result.push(curr);
            state = curr;
            prev_heart_offset = heart_offset_val;
            prev_friction = friction_val;
            index += 1;
        }

        result
    }
}

#[allow(clippy::too_many_arguments)]
fn step_curved(
    prev: &Point,
    axis: f32,
    delta_angle: f32,
    roll_speed_val: f32,
    heart_offset_val: f32,
    friction_val: f32,
    resistance_val: f32,
    driven: bool,
) -> Point {
    let prev_frame = prev.frame();

    let axis_rad = axis.to_radians();
    let curve_axis = prev.normal * -axis_rad.cos() + prev.lateral * axis_rad.sin();
    let curve_quat = Quaternion::from_axis_angle(curve_axis, delta_angle.to_radians());
    let curr_direction = curve_quat.mul_vec(prev.direction).normalize();

    let original_roll_deg = prev.lateral.y.atan2(-prev.normal.y).to_degrees();
    let original_roll = ((original_roll_deg + 540.0) % 360.0) - 180.0;

    let up = Float3::new(0.0, 1.0, 0.0);
    let right = Float3::new(1.0, 0.0, 0.0);

    let mut curr_lateral = curr_direction.cross(up).normalize();
    if curr_lateral.magnitude() < sim::EPSILON {
        curr_lateral = right;
    }
    let mut curr_normal = curr_direction.cross(curr_lateral).normalize();

    if original_roll.abs() > sim::EPSILON {
        let preserved_roll_quat = Quaternion::from_axis_angle(curr_direction, -original_roll.to_radians());
        curr_lateral = preserved_roll_quat.mul_vec(curr_lateral).normalize();
        curr_normal = curr_direction.cross(curr_lateral).normalize();
    }

    let half_step_distance = prev.velocity / (2.0 * sim::HZ);
    let prev_heart_pos = prev.spine_position + prev.normal * heart_offset_val;
    let curr_heart_pos_if_spine_static = prev.spine_position + curr_normal * heart_offset_val;

    let curr_spine_position = prev.spine_position
        + curr_direction * half_step_distance
        + prev.direction * half_step_distance
        + (prev_heart_pos - curr_heart_pos_if_spine_static);

    let delta_roll = roll_speed_val / sim::HZ;
    let roll_quat = Quaternion::from_axis_angle(curr_direction, -delta_roll);
    curr_lateral = roll_quat.mul_vec(curr_lateral).normalize();
    curr_normal = curr_direction.cross(curr_lateral).normalize();

    let heart_advance = (curr_spine_position + curr_normal * heart_offset_val - prev_heart_pos).magnitude();
    let new_heart_arc = prev.heart_arc + heart_advance;
    let spine_advance = (curr_spine_position - prev.spine_position).magnitude();
    let new_spine_arc = prev.spine_arc + spine_advance;

    let mut new_energy = prev.energy;
    let mut new_velocity = prev.velocity;

    if !driven {
        let center_y = (curr_spine_position + curr_normal * (0.9 * heart_offset_val)).y;
        let friction_distance = new_heart_arc - prev.friction_origin;
        (new_energy, new_velocity) = sim::update_energy(
            prev.energy,
            prev.velocity,
            center_y,
            friction_distance,
            friction_val,
            resistance_val,
        );
    }

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
    fn curved_node_basic_arc() {
        let anchor = Point::new(
            Float3::new(0.0, 0.0, 0.0),
            Float3::new(1.0, 0.0, 0.0),
            Float3::new(0.0, 1.0, 0.0),
            Float3::new(0.0, 0.0, 1.0),
            10.0,
            0.5 * 10.0 * 10.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        );

        let result = CurvedNode::build(
            &anchor,
            10.0,
            90.0,
            0.0,
            0.0,
            0.0,
            true,
            &[],
            &[Keyframe::simple(0.0, 10.0)],
            &[Keyframe::simple(0.0, 0.0)],
            &[Keyframe::simple(0.0, 0.0)],
            &[Keyframe::simple(0.0, 0.0)],
            0.0,
            0.0,
            0.0,
        );

        assert!(result.len() > 1);
        assert!((result[0].spine_position - anchor.spine_position).magnitude() < 0.01);
    }

    #[test]
    fn curved_node_with_lead_in() {
        let anchor = Point::new(
            Float3::new(0.0, 0.0, 0.0),
            Float3::new(1.0, 0.0, 0.0),
            Float3::new(0.0, 1.0, 0.0),
            Float3::new(0.0, 0.0, 1.0),
            10.0,
            0.5 * 10.0 * 10.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        );

        let result = CurvedNode::build(
            &anchor,
            10.0,
            45.0,
            0.0,
            10.0,
            0.0,
            true,
            &[],
            &[Keyframe::simple(0.0, 10.0)],
            &[Keyframe::simple(0.0, 0.0)],
            &[Keyframe::simple(0.0, 0.0)],
            &[Keyframe::simple(0.0, 0.0)],
            0.0,
            0.0,
            0.0,
        );

        assert!(result.len() > 1);
    }

    #[test]
    fn curved_node_with_lead_out() {
        let anchor = Point::new(
            Float3::new(0.0, 0.0, 0.0),
            Float3::new(1.0, 0.0, 0.0),
            Float3::new(0.0, 1.0, 0.0),
            Float3::new(0.0, 0.0, 1.0),
            10.0,
            0.5 * 10.0 * 10.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        );

        let result = CurvedNode::build(
            &anchor,
            10.0,
            45.0,
            0.0,
            0.0,
            10.0,
            true,
            &[],
            &[Keyframe::simple(0.0, 10.0)],
            &[Keyframe::simple(0.0, 0.0)],
            &[Keyframe::simple(0.0, 0.0)],
            &[Keyframe::simple(0.0, 0.0)],
            0.0,
            0.0,
            0.0,
        );

        assert!(result.len() > 1);
    }

    mod golden_tests {
        use super::*;
        use crate::golden::GoldTrackData;
        use crate::point_comparer::assert_points_match_gold;

        fn build_from_gold_section(section: &crate::golden::GoldSection) -> Vec<Point> {
            let anchor = section.inputs.anchor.to_point();
            let curve_data = section.inputs.curve_data.as_ref().unwrap();
            let keyframes = section.inputs.keyframes.as_ref();
            let overrides = section.inputs.property_overrides.as_ref();

            let roll_speed: Vec<_> = keyframes
                .map(|k| k.roll_speed.iter().map(|kf| kf.to_keyframe()).collect())
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

            CurvedNode::build(
                &anchor,
                curve_data.radius,
                curve_data.arc,
                curve_data.axis,
                curve_data.lead_in,
                curve_data.lead_out,
                overrides.map(|o| o.fixed_velocity).unwrap_or(false),
                &roll_speed,
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
        fn veloci_curved_section1_matches_gold() {
            let data = GoldTrackData::load("../../Assets/Tests/TrackData/veloci.json")
                .expect("Failed to load veloci.json");
            let sections = data.get_curved_sections();
            assert!(!sections.is_empty(), "No curved sections found in veloci.json");

            let section = sections[0];
            let result = build_from_gold_section(section);

            assert_points_match_gold(&result, &section.outputs.points);
        }
    }
}
