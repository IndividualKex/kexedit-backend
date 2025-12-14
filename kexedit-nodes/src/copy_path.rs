use kexedit_core::{
    evaluate, sim, Frame, Float3, Keyframe, Point,
};

const MAX_ITERATIONS: usize = 1_000_000;

pub struct CopyPathNode;

impl CopyPathNode {
    #[allow(clippy::too_many_arguments)]
    pub fn build(
        anchor: &Point,
        source_path: &[Point],
        start: f32,
        end: f32,
        driven: bool,
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

        if source_path.len() < 2 {
            return result;
        }

        let start_index = if start <= 0.0 {
            0
        } else {
            ((start * sim::HZ).round() as usize).clamp(0, source_path.len() - 1)
        };

        let end_index = if end < 0.0 {
            source_path.len() - 1
        } else {
            ((end * sim::HZ).round() as usize).clamp(start_index, source_path.len() - 1)
        };

        if end_index - start_index < 1 {
            return result;
        }

        let path_start = source_path[start_index];
        let path_basis = Matrix3::from_columns(path_start.lateral, path_start.normal, path_start.direction);
        let anchor_basis = Matrix3::from_columns(anchor.lateral, anchor.normal, anchor.direction);
        let rotation = anchor_basis.multiply(&path_basis.transpose());
        let translation = anchor.spine_position - rotation.multiply_vector(path_start.spine_position);

        let mut distance = source_path[start_index].heart_arc;
        let end_distance = source_path[end_index].heart_arc;
        let mut index = 0;
        let mut iters = 0;

        let mut state = *anchor;
        let mut prev_heart_offset = anchor_heart;
        let mut prev_friction = anchor_friction;

        if !driven && anchor.velocity < sim::MIN_VELOCITY && anchor.frame().pitch() < 0.0 {
            let center_y = anchor.frame().heart_position(anchor.spine_position, prev_heart_offset * 0.9).y;
            let energy = 0.5 * sim::MIN_VELOCITY * sim::MIN_VELOCITY + sim::G * center_y;
            state = Point::new(
                anchor.spine_position,
                anchor.direction,
                anchor.normal,
                anchor.lateral,
                sim::MIN_VELOCITY,
                energy,
                anchor.normal_force,
                anchor.lateral_force,
                anchor.heart_arc,
                anchor.spine_arc,
                anchor.spine_advance,
                anchor.heart_arc,
                0.0,
                0.0,
                0.0,
                0.0,
            );
        }

        while distance < end_distance {
            if iters > MAX_ITERATIONS {
                break;
            }
            iters += 1;

            let mut prev = state;
            let t = index as f32 / sim::HZ;

            let mut advance_velocity = prev.velocity;
            if driven {
                let velocity = evaluate(driven_velocity, t, prev.velocity);
                if velocity < sim::MIN_VELOCITY {
                    break;
                }
                prev = prev.with_velocity(velocity, prev_heart_offset, prev_friction, true);
                advance_velocity = velocity;
            } else if prev.velocity < sim::MIN_VELOCITY {
                if prev.frame().pitch() < 0.0 {
                    prev = prev.with_velocity(sim::MIN_VELOCITY, prev_heart_offset, prev_friction, true);
                    advance_velocity = sim::MIN_VELOCITY;
                } else {
                    break;
                }
            }

            let heart_offset_val = evaluate(heart_offset, t, anchor_heart);
            let friction_val = evaluate(friction, t, anchor_friction);
            let resistance_val = evaluate(resistance, t, anchor_resistance);

            let expected_advancement = advance_velocity / sim::HZ;
            let desired_distance = distance + expected_advancement;
            let (start_point, end_point, interp_t) = project(source_path, &mut index, desired_distance, start_index, end_index);

            let (mut position, mut direction, mut lateral, mut normal);
            if interp_t < 0.0 {
                position = end_point.spine_position;
                direction = end_point.direction;
                lateral = end_point.lateral;
                normal = end_point.normal;
            } else {
                position = start_point.spine_position + (end_point.spine_position - start_point.spine_position) * interp_t;
                direction = (start_point.direction + (end_point.direction - start_point.direction) * interp_t).normalize();
                lateral = (start_point.lateral + (end_point.lateral - start_point.lateral) * interp_t).normalize();
                normal = (start_point.normal + (end_point.normal - start_point.normal) * interp_t).normalize();
            }

            position = rotation.multiply_vector(position) + translation;
            direction = rotation.multiply_vector(direction);
            lateral = rotation.multiply_vector(lateral);
            normal = rotation.multiply_vector(normal);

            let curr_frame = Frame::new(direction, normal, lateral);
            let curr_heart_pos = curr_frame.heart_position(position, heart_offset_val);
            let prev_heart_pos = prev.frame().heart_position(prev.spine_position, prev_heart_offset);
            let spine_advance = (curr_heart_pos - prev_heart_pos).magnitude();
            let heart_advance = (position - prev.spine_position).magnitude();
            let heart_arc = prev.heart_arc + spine_advance;
            let spine_arc = prev.spine_arc + heart_advance;

            distance += expected_advancement;

            let center_y = curr_frame.heart_position(position, heart_offset_val * 0.9).y;
            let friction_distance = heart_arc - state.friction_origin;

            let (new_energy, new_velocity);
            if driven {
                new_velocity = evaluate(driven_velocity, t, prev.velocity);
                let prev_center_y = prev.frame().heart_position(prev.spine_position, prev_heart_offset * 0.9).y;
                new_energy = 0.5 * new_velocity * new_velocity + sim::G * prev_center_y;
            } else {
                (new_energy, new_velocity) = sim::update_energy(
                    prev.energy,
                    prev.velocity,
                    center_y,
                    friction_distance,
                    friction_val,
                    resistance_val,
                );
            }

            let force_vec = compute_force_vector(&prev, &curr_frame, position, heart_advance, new_velocity);
            let normal_force = -force_vec.dot(normal);
            let lateral_force = -force_vec.dot(lateral);

            state = Point::new(
                position,
                direction,
                normal,
                lateral,
                new_velocity,
                new_energy,
                normal_force,
                lateral_force,
                heart_arc,
                spine_arc,
                spine_advance,
                prev.friction_origin,
                anchor.roll_speed,
                heart_offset_val,
                friction_val,
                resistance_val,
            );

            result.push(state);
            prev_heart_offset = heart_offset_val;
            prev_friction = friction_val;
        }

        result
    }
}

struct Matrix3 {
    c0: Float3,
    c1: Float3,
    c2: Float3,
}

impl Matrix3 {
    fn from_columns(c0: Float3, c1: Float3, c2: Float3) -> Self {
        Self { c0, c1, c2 }
    }

    fn transpose(&self) -> Self {
        Self {
            c0: Float3::new(self.c0.x, self.c1.x, self.c2.x),
            c1: Float3::new(self.c0.y, self.c1.y, self.c2.y),
            c2: Float3::new(self.c0.z, self.c1.z, self.c2.z),
        }
    }

    fn multiply(&self, other: &Matrix3) -> Self {
        Self {
            c0: Float3::new(
                self.c0.x * other.c0.x + self.c1.x * other.c0.y + self.c2.x * other.c0.z,
                self.c0.y * other.c0.x + self.c1.y * other.c0.y + self.c2.y * other.c0.z,
                self.c0.z * other.c0.x + self.c1.z * other.c0.y + self.c2.z * other.c0.z,
            ),
            c1: Float3::new(
                self.c0.x * other.c1.x + self.c1.x * other.c1.y + self.c2.x * other.c1.z,
                self.c0.y * other.c1.x + self.c1.y * other.c1.y + self.c2.y * other.c1.z,
                self.c0.z * other.c1.x + self.c1.z * other.c1.y + self.c2.z * other.c1.z,
            ),
            c2: Float3::new(
                self.c0.x * other.c2.x + self.c1.x * other.c2.y + self.c2.x * other.c2.z,
                self.c0.y * other.c2.x + self.c1.y * other.c2.y + self.c2.y * other.c2.z,
                self.c0.z * other.c2.x + self.c1.z * other.c2.y + self.c2.z * other.c2.z,
            ),
        }
    }

    fn multiply_vector(&self, v: Float3) -> Float3 {
        Float3::new(
            self.c0.x * v.x + self.c1.x * v.y + self.c2.x * v.z,
            self.c0.y * v.x + self.c1.y * v.y + self.c2.y * v.z,
            self.c0.z * v.x + self.c1.z * v.y + self.c2.z * v.z,
        )
    }
}

fn project(
    source_path: &[Point],
    index: &mut usize,
    distance: f32,
    start_index: usize,
    end_index: usize,
) -> (Point, Point, f32) {
    if distance >= source_path[end_index].heart_arc {
        *index = end_index - start_index;
        let end = source_path[end_index];
        return (end, end, -1.0);
    }

    for i in (start_index + *index)..end_index {
        if source_path[i + 1].heart_arc >= distance {
            *index = i - start_index;
            break;
        }
    }

    let start = source_path[start_index + *index];
    let end = source_path[start_index + *index + 1];

    let denom = end.heart_arc - start.heart_arc;
    let t = if denom > sim::EPSILON {
        ((distance - start.heart_arc) / denom).clamp(0.0, 1.0)
    } else {
        0.0
    };

    (start, end, t)
}

fn compute_force_vector(
    prev: &Point,
    curr: &Frame,
    _curr_position: Float3,
    heart_advance: f32,
    velocity: f32,
) -> Float3 {
    let roll = curr.roll();
    let pitch = curr.pitch();
    let yaw = curr.yaw();
    let prev_pitch = prev.frame().pitch();
    let prev_yaw = prev.frame().yaw();

    let pitch_from_last = sim::wrap_angle(pitch - prev_pitch);
    let yaw_from_last = sim::wrap_angle(yaw - prev_yaw);
    let yaw_scale_factor = pitch.abs().cos();
    let angle_from_last = (yaw_scale_factor * yaw_scale_factor * yaw_from_last * yaw_from_last
        + pitch_from_last * pitch_from_last)
        .sqrt();

    if angle_from_last.abs() < sim::EPSILON {
        return Float3::new(0.0, 1.0, 0.0);
    }

    let cos_roll = roll.cos();
    let sin_roll = roll.sin();
    let normal_angle = -pitch_from_last * cos_roll - yaw_scale_factor * yaw_from_last * sin_roll;
    let lateral_angle = pitch_from_last * sin_roll - yaw_scale_factor * yaw_from_last * cos_roll;

    Float3::new(0.0, 1.0, 0.0)
        + curr.lateral * (velocity * sim::HZ * lateral_angle / sim::G)
        + curr.normal * (heart_advance * sim::HZ * sim::HZ * normal_angle / sim::G)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn copy_path_node_basic() {
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

        let source_path = vec![
            Point::new(
                Float3::new(0.0, 0.0, 0.0),
                Float3::new(1.0, 0.0, 0.0),
                Float3::new(0.0, 1.0, 0.0),
                Float3::new(0.0, 0.0, 1.0),
                10.0,
                50.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ),
            Point::new(
                Float3::new(1.0, 0.0, 0.0),
                Float3::new(1.0, 0.0, 0.0),
                Float3::new(0.0, 1.0, 0.0),
                Float3::new(0.0, 0.0, 1.0),
                10.0,
                50.0,
                1.0,
                0.0,
                1.0,
                1.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ),
        ];

        let result = CopyPathNode::build(
            &anchor,
            &source_path,
            0.0,
            1.0,
            true,
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
    fn copy_path_node_empty_source() {
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

        let result = CopyPathNode::build(
            &anchor,
            &[],
            0.0,
            1.0,
            true,
            &[Keyframe::simple(0.0, 10.0)],
            &[Keyframe::simple(0.0, 0.0)],
            &[Keyframe::simple(0.0, 0.0)],
            &[Keyframe::simple(0.0, 0.0)],
            0.0,
            0.0,
            0.0,
        );

        assert_eq!(result.len(), 1);
    }

    mod golden_tests {
        use super::*;
        use crate::golden::GoldTrackData;
        use crate::point_comparer::assert_points_match_gold;

        fn build_from_gold_section(section: &crate::golden::GoldSection) -> Vec<Point> {
            let anchor = section.inputs.anchor.to_point();
            let source_path: Vec<Point> = section.inputs.source_path.iter().map(|p| p.to_point()).collect();
            let keyframes = section.inputs.keyframes.as_ref();
            let overrides = section.inputs.property_overrides.as_ref();

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

            CopyPathNode::build(
                &anchor,
                &source_path,
                section.inputs.start,
                section.inputs.end,
                overrides.map(|o| o.fixed_velocity).unwrap_or(false),
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
        fn all_types_copy_path_section1_matches_gold() {
            let data = GoldTrackData::load("../test-data/all_types.json")
                .expect("Failed to load all_types.json");
            let sections = data.get_copy_path_sections();

            if sections.is_empty() {
                eprintln!("No CopyPath sections found in all_types.json - skipping test");
                return;
            }

            let section = sections[0];
            let result = build_from_gold_section(section);

            assert_points_match_gold(&result, &section.outputs.points);
        }

        #[test]
        fn all_types_copy_path_section2_matches_gold() {
            let data = GoldTrackData::load("../test-data/all_types.json")
                .expect("Failed to load all_types.json");
            let sections = data.get_copy_path_sections();

            if sections.len() < 2 {
                eprintln!("Less than 2 CopyPath sections in all_types.json - skipping test");
                return;
            }

            let section = sections[1];
            let result = build_from_gold_section(section);

            assert_points_match_gold(&result, &section.outputs.points);
        }

        #[test]
        fn all_types_copy_path_section3_matches_gold() {
            let data = GoldTrackData::load("../test-data/all_types.json")
                .expect("Failed to load all_types.json");
            let sections = data.get_copy_path_sections();

            if sections.len() < 3 {
                eprintln!("Less than 3 CopyPath sections in all_types.json - skipping test");
                return;
            }

            let section = sections[2];
            let result = build_from_gold_section(section);

            assert_points_match_gold(&result, &section.outputs.points);
        }
    }
}
