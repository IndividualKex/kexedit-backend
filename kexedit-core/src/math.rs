use std::ops::{Add, Mul, Neg, Sub};

/// 3D vector with f32 components.
/// C-compatible layout for FFI.
#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Float3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Float3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);
    pub const UP: Self = Self::new(0.0, 1.0, 0.0);
    pub const DOWN: Self = Self::new(0.0, -1.0, 0.0);
    pub const RIGHT: Self = Self::new(1.0, 0.0, 0.0);
    pub const BACK: Self = Self::new(0.0, 0.0, -1.0);

    pub fn magnitude(self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(self) -> Self {
        let mag = self.magnitude();
        if mag < f32::EPSILON {
            return Self::ZERO;
        }
        self * (1.0 / mag)
    }

    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }
}

impl Add for Float3 {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl Sub for Float3 {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}

impl Mul<f32> for Float3 {
    type Output = Self;
    fn mul(self, scalar: f32) -> Self {
        Self::new(self.x * scalar, self.y * scalar, self.z * scalar)
    }
}

impl Neg for Float3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

/// Unit quaternion for 3D rotations.
/// C-compatible layout for FFI.
#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Quaternion {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Quaternion {
    pub const fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    pub const IDENTITY: Self = Self::new(0.0, 0.0, 0.0, 1.0);

    pub fn from_axis_angle(axis: Float3, angle: f32) -> Self {
        let half_angle = angle * 0.5;
        let s = half_angle.sin();
        let c = half_angle.cos();
        let normalized = axis.normalize();

        Self::new(normalized.x * s, normalized.y * s, normalized.z * s, c)
    }

    pub fn mul_vec(self, v: Float3) -> Float3 {
        let qv = Float3::new(self.x, self.y, self.z);
        let uv = qv.cross(v);
        let uuv = qv.cross(uv);
        v + (uv * (2.0 * self.w)) + (uuv * 2.0)
    }

    pub fn mul(self, other: Self) -> Self {
        Self::new(
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_float3_normalize() {
        let v = Float3::new(3.0, 4.0, 0.0);
        let normalized = v.normalize();
        assert_relative_eq!(normalized.x, 0.6, epsilon = 1e-6);
        assert_relative_eq!(normalized.y, 0.8, epsilon = 1e-6);
        assert_relative_eq!(normalized.magnitude(), 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_float3_cross() {
        let a = Float3::new(1.0, 0.0, 0.0);
        let b = Float3::new(0.0, 1.0, 0.0);
        let c = a.cross(b);
        assert_relative_eq!(c.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(c.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(c.z, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_quaternion_axis_angle() {
        use std::f32::consts::PI;
        let axis = Float3::UP;
        let angle = PI / 2.0;
        let q = Quaternion::from_axis_angle(axis, angle);

        let v = Float3::RIGHT;
        let rotated = q.mul_vec(v);

        assert_relative_eq!(rotated.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(rotated.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(rotated.z, -1.0, epsilon = 1e-6);
    }
}
