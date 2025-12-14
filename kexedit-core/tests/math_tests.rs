use kexedit_core::{Float3, Quaternion};
use std::f32::consts::PI;

#[test]
fn float3_operations() {
    let a = Float3::new(1.0, 2.0, 3.0);
    let b = Float3::new(4.0, 5.0, 6.0);

    let sum = a + b;
    assert!((sum.x - 5.0).abs() < 1e-6);
    assert!((sum.y - 7.0).abs() < 1e-6);
    assert!((sum.z - 9.0).abs() < 1e-6);

    let diff = b - a;
    assert!((diff.x - 3.0).abs() < 1e-6);
    assert!((diff.y - 3.0).abs() < 1e-6);
    assert!((diff.z - 3.0).abs() < 1e-6);

    let scaled = a * 2.0;
    assert!((scaled.x - 2.0).abs() < 1e-6);
    assert!((scaled.y - 4.0).abs() < 1e-6);
    assert!((scaled.z - 6.0).abs() < 1e-6);
}

#[test]
fn float3_normalize_unit_vectors() {
    let vectors = [
        Float3::new(1.0, 0.0, 0.0),
        Float3::new(0.0, 1.0, 0.0),
        Float3::new(0.0, 0.0, 1.0),
    ];

    for v in vectors {
        let normalized = v.normalize();
        assert!((normalized.magnitude() - 1.0).abs() < 1e-6);
    }
}

#[test]
fn float3_cross_product_right_hand_rule() {
    let x = Float3::new(1.0, 0.0, 0.0);
    let y = Float3::new(0.0, 1.0, 0.0);
    let z = x.cross(y);

    assert!((z.x - 0.0).abs() < 1e-6);
    assert!((z.y - 0.0).abs() < 1e-6);
    assert!((z.z - 1.0).abs() < 1e-6);
}

#[test]
fn quaternion_identity_preserves_vector() {
    let v = Float3::new(1.0, 2.0, 3.0);
    let q = Quaternion::IDENTITY;
    let result = q.mul_vec(v);

    assert!((result.x - v.x).abs() < 1e-6);
    assert!((result.y - v.y).abs() < 1e-6);
    assert!((result.z - v.z).abs() < 1e-6);
}

#[test]
fn quaternion_rotation_90_degrees() {
    let v = Float3::new(1.0, 0.0, 0.0);
    let axis = Float3::new(0.0, 1.0, 0.0);
    let q = Quaternion::from_axis_angle(axis, PI / 2.0);
    let result = q.mul_vec(v);

    assert!((result.x - 0.0).abs() < 1e-6);
    assert!((result.y - 0.0).abs() < 1e-6);
    assert!((result.z - -1.0).abs() < 1e-6);
}
