use crate::{sim, Float3, Frame};

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Point {
    pub spine_position: Float3,
    pub direction: Float3,
    pub normal: Float3,
    pub lateral: Float3,
    pub velocity: f32,
    pub energy: f32,
    pub normal_force: f32,
    pub lateral_force: f32,
    pub heart_arc: f32,
    pub spine_arc: f32,
    pub spine_advance: f32,
    pub friction_origin: f32,
    pub roll_speed: f32,
    pub heart_offset: f32,
    pub friction: f32,
    pub resistance: f32,
}

impl Point {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        spine_position: Float3,
        direction: Float3,
        normal: Float3,
        lateral: Float3,
        velocity: f32,
        energy: f32,
        normal_force: f32,
        lateral_force: f32,
        heart_arc: f32,
        spine_arc: f32,
        spine_advance: f32,
        friction_origin: f32,
        roll_speed: f32,
        heart_offset: f32,
        friction: f32,
        resistance: f32,
    ) -> Self {
        Self {
            spine_position,
            direction,
            normal,
            lateral,
            velocity,
            energy,
            normal_force,
            lateral_force,
            heart_arc,
            spine_arc,
            spine_advance,
            friction_origin,
            roll_speed,
            heart_offset,
            friction,
            resistance,
        }
    }

    pub fn roll(&self) -> f32 {
        self.lateral.y.atan2(-self.normal.y)
    }

    pub fn frame(&self) -> Frame {
        Frame::new(self.direction, self.normal, self.lateral)
    }

    pub fn heart_position(&self, offset: f32) -> Float3 {
        self.spine_position + self.normal * offset
    }

    pub fn create(
        spine_position: Float3,
        direction: Float3,
        roll: f32,
        velocity: f32,
        heart_offset: f32,
        friction: f32,
        resistance: f32,
    ) -> Self {
        let frame = from_direction_and_roll(direction, roll);
        let heart_pos = frame.heart_position(spine_position, heart_offset);
        let energy = 0.5 * velocity * velocity + sim::G * heart_pos.y;

        Self::new(
            spine_position,
            frame.direction,
            frame.normal,
            frame.lateral,
            velocity,
            energy,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            heart_offset,
            friction,
            resistance,
        )
    }

    pub const DEFAULT: Self = Self::new(
        Float3::new(0.0, 3.0, 0.0),
        Float3::BACK,
        Float3::DOWN,
        Float3::RIGHT,
        10.0,
        0.5 * 10.0 * 10.0 + sim::G * 3.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.1,
        0.0,
        0.0,
    );

    pub fn with_friction_origin(&self, new_origin: f32) -> Self {
        Self::new(
            self.spine_position,
            self.direction,
            self.normal,
            self.lateral,
            self.velocity,
            self.energy,
            self.normal_force,
            self.lateral_force,
            self.heart_arc,
            self.spine_arc,
            self.spine_advance,
            new_origin,
            self.roll_speed,
            self.heart_offset,
            self.friction,
            self.resistance,
        )
    }

    pub fn with_velocity_and_energy(
        &self,
        new_velocity: f32,
        new_energy: f32,
        new_friction_origin: f32,
    ) -> Self {
        Self::new(
            self.spine_position,
            self.direction,
            self.normal,
            self.lateral,
            new_velocity,
            new_energy,
            self.normal_force,
            self.lateral_force,
            self.heart_arc,
            self.spine_arc,
            self.spine_advance,
            new_friction_origin,
            self.roll_speed,
            self.heart_offset,
            self.friction,
            self.resistance,
        )
    }

    pub fn with_forces(&self, new_normal_force: f32, new_lateral_force: f32) -> Self {
        Self::new(
            self.spine_position,
            self.direction,
            self.normal,
            self.lateral,
            self.velocity,
            self.energy,
            new_normal_force,
            new_lateral_force,
            self.heart_arc,
            self.spine_arc,
            self.spine_advance,
            self.friction_origin,
            self.roll_speed,
            self.heart_offset,
            self.friction,
            self.resistance,
        )
    }

    pub fn with_velocity(
        &self,
        new_velocity: f32,
        heart_offset: f32,
        friction: f32,
        reset_friction: bool,
    ) -> Self {
        let new_friction_origin = if reset_friction {
            self.heart_arc
        } else {
            self.friction_origin
        };
        let center_y = self.heart_position(heart_offset * 0.9).y;
        let friction_distance = self.heart_arc - new_friction_origin;
        let new_energy = sim::compute_total_energy(new_velocity, center_y, friction_distance, friction);

        Self::new(
            self.spine_position,
            self.direction,
            self.normal,
            self.lateral,
            new_velocity,
            new_energy,
            self.normal_force,
            self.lateral_force,
            self.heart_arc,
            self.spine_arc,
            self.spine_advance,
            new_friction_origin,
            self.roll_speed,
            self.heart_offset,
            self.friction,
            self.resistance,
        )
    }
}

fn from_direction_and_roll(direction: Float3, roll: f32) -> Frame {
    let dir = direction.normalize();
    let yaw = (-dir.x).atan2(-dir.z);

    let yaw_quat = crate::Quaternion::from_axis_angle(Float3::UP, yaw);
    let lateral_base = yaw_quat.mul_vec(Float3::RIGHT);
    let roll_quat = crate::Quaternion::from_axis_angle(dir, -roll);
    let lateral = roll_quat.mul_vec(lateral_base).normalize();
    let normal = dir.cross(lateral).normalize();

    Frame::new(dir, normal, lateral)
}

impl Frame {
    pub fn heart_position(&self, spine_position: Float3, offset: f32) -> Float3 {
        spine_position + self.normal * offset
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    const TOLERANCE: f32 = 1e-5;

    #[test]
    fn default_point_has_expected_values() {
        let point = Point::DEFAULT;
        assert_relative_eq!(point.spine_position.y, 3.0, epsilon = TOLERANCE);
        assert_relative_eq!(point.velocity, 10.0, epsilon = TOLERANCE);
        assert_relative_eq!(point.heart_offset, 1.1, epsilon = TOLERANCE);
    }

    #[test]
    fn create_point_sets_energy_correctly() {
        let spine_pos = Float3::new(0.0, 5.0, 0.0);
        let velocity = 15.0;
        let heart_offset = 1.0;

        let point = Point::create(spine_pos, Float3::BACK, 0.0, velocity, heart_offset, 0.0, 0.0);

        let expected_heart_y = spine_pos.y - heart_offset;
        let expected_energy = 0.5 * velocity * velocity + sim::G * expected_heart_y;

        assert_relative_eq!(point.energy, expected_energy, epsilon = TOLERANCE);
    }

    #[test]
    fn with_friction_origin_updates_only_friction_origin() {
        let point = Point::DEFAULT;
        let new_origin = 42.0;
        let updated = point.with_friction_origin(new_origin);

        assert_relative_eq!(updated.friction_origin, new_origin, epsilon = TOLERANCE);
        assert_relative_eq!(updated.velocity, point.velocity, epsilon = TOLERANCE);
    }

    #[test]
    fn with_forces_updates_only_forces() {
        let point = Point::DEFAULT;
        let new_normal = 2.5;
        let new_lateral = 0.8;
        let updated = point.with_forces(new_normal, new_lateral);

        assert_relative_eq!(updated.normal_force, new_normal, epsilon = TOLERANCE);
        assert_relative_eq!(updated.lateral_force, new_lateral, epsilon = TOLERANCE);
        assert_relative_eq!(updated.velocity, point.velocity, epsilon = TOLERANCE);
    }
}
