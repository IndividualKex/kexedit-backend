use crate::math::{Float3, Quaternion};

/// Orthonormal coordinate frame for track orientation.
///
/// Represents a right-handed coordinate system with three orthogonal unit vectors:
/// - `direction`: Forward direction along track (tangent)
/// - `normal`: Upward direction perpendicular to track (binormal)
/// - `lateral`: Rightward direction (cross product of direction and normal)
///
/// C-compatible layout for FFI.
#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Frame {
    pub direction: Float3,
    pub normal: Float3,
    pub lateral: Float3,
}

impl Frame {
    pub const fn new(direction: Float3, normal: Float3, lateral: Float3) -> Self {
        Self {
            direction,
            normal,
            lateral,
        }
    }

    /// Rotates the frame around an arbitrary axis by the given angle.
    ///
    /// # Arguments
    /// * `axis` - Rotation axis (will be normalized)
    /// * `angle` - Rotation angle in radians
    pub fn rotate_around(self, axis: Float3, angle: f32) -> Self {
        let q = Quaternion::from_axis_angle(axis, angle);
        Self {
            direction: q.mul_vec(self.direction).normalize(),
            normal: q.mul_vec(self.normal).normalize(),
            lateral: q.mul_vec(self.lateral).normalize(),
        }
    }

    pub fn roll(self) -> f32 {
        self.lateral.y.atan2(-self.normal.y)
    }

    pub fn pitch(self) -> f32 {
        let mag = (self.direction.x * self.direction.x + self.direction.z * self.direction.z).sqrt();
        self.direction.y.atan2(mag)
    }

    pub fn yaw(self) -> f32 {
        (-self.direction.x).atan2(-self.direction.z)
    }

    pub fn with_roll(self, delta_roll: f32) -> Self {
        let q = Quaternion::from_axis_angle(self.direction, -delta_roll);
        let new_lateral = q.mul_vec(self.lateral).normalize();
        let new_normal = self.direction.cross(new_lateral).normalize();
        Self::new(self.direction, new_normal, new_lateral)
    }

    pub fn with_pitch(self, delta_pitch: f32) -> Self {
        let up = if self.normal.y >= 0.0 {
            Float3::UP
        } else {
            -Float3::UP
        };
        let pitch_axis = up.cross(self.direction).normalize();
        let pitch_quat = Quaternion::from_axis_angle(pitch_axis, delta_pitch);
        let new_direction = pitch_quat.mul_vec(self.direction).normalize();
        let new_lateral = pitch_quat.mul_vec(self.lateral).normalize();
        let new_normal = new_direction.cross(new_lateral).normalize();
        Self::new(new_direction, new_normal, new_lateral)
    }

    pub fn with_yaw(self, delta_yaw: f32) -> Self {
        let yaw_quat = Quaternion::from_axis_angle(Float3::UP, delta_yaw);
        let new_direction = yaw_quat.mul_vec(self.direction).normalize();
        let new_lateral = yaw_quat.mul_vec(self.lateral).normalize();
        let new_normal = new_direction.cross(new_lateral).normalize();
        Self::new(new_direction, new_normal, new_lateral)
    }

    pub const DEFAULT: Self = Self::new(Float3::BACK, Float3::DOWN, Float3::RIGHT);
}

impl Default for Frame {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f32::consts::PI;

    #[test]
    fn test_frame_rotate_around() {
        let frame = Frame::DEFAULT;
        let axis = Float3::UP;
        let angle = PI / 2.0;

        let rotated = frame.rotate_around(axis, angle);

        assert_relative_eq!(rotated.direction.x, -1.0, epsilon = 1e-6);
        assert_relative_eq!(rotated.direction.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(rotated.direction.z, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_frame_with_roll() {
        let frame = Frame::DEFAULT;
        let rolled = frame.with_roll(PI / 4.0);

        let expected_roll = PI / 4.0;
        assert_relative_eq!(rolled.roll(), expected_roll, epsilon = 1e-6);
    }

    #[test]
    fn test_frame_orthonormal() {
        let frame = Frame::DEFAULT;

        let dir_mag = frame.direction.magnitude();
        let normal_mag = frame.normal.magnitude();
        let lateral_mag = frame.lateral.magnitude();

        assert_relative_eq!(dir_mag, 1.0, epsilon = 1e-6);
        assert_relative_eq!(normal_mag, 1.0, epsilon = 1e-6);
        assert_relative_eq!(lateral_mag, 1.0, epsilon = 1e-6);

        let dot_dn = frame.direction.dot(frame.normal);
        let dot_dl = frame.direction.dot(frame.lateral);
        let dot_nl = frame.normal.dot(frame.lateral);

        assert_relative_eq!(dot_dn, 0.0, epsilon = 1e-6);
        assert_relative_eq!(dot_dl, 0.0, epsilon = 1e-6);
        assert_relative_eq!(dot_nl, 0.0, epsilon = 1e-6);
    }
}
