use kexedit_core::{Float3, Frame};
use std::f32::consts::PI;

#[test]
fn frame_default_is_orthonormal() {
    let frame = Frame::DEFAULT;

    let dir_mag = frame.direction.magnitude();
    let normal_mag = frame.normal.magnitude();
    let lateral_mag = frame.lateral.magnitude();

    assert!((dir_mag - 1.0).abs() < 1e-6);
    assert!((normal_mag - 1.0).abs() < 1e-6);
    assert!((lateral_mag - 1.0).abs() < 1e-6);

    let dot_dn = frame.direction.dot(frame.normal);
    let dot_dl = frame.direction.dot(frame.lateral);
    let dot_nl = frame.normal.dot(frame.lateral);

    assert!(dot_dn.abs() < 1e-6);
    assert!(dot_dl.abs() < 1e-6);
    assert!(dot_nl.abs() < 1e-6);
}

#[test]
fn rotate_around_preserves_orthonormality() {
    let frame = Frame::DEFAULT;
    let rotated = frame.rotate_around(Float3::UP, PI / 4.0);

    let dir_mag = rotated.direction.magnitude();
    let normal_mag = rotated.normal.magnitude();
    let lateral_mag = rotated.lateral.magnitude();

    assert!((dir_mag - 1.0).abs() < 1e-6);
    assert!((normal_mag - 1.0).abs() < 1e-6);
    assert!((lateral_mag - 1.0).abs() < 1e-6);

    let dot_dn = rotated.direction.dot(rotated.normal);
    let dot_dl = rotated.direction.dot(rotated.lateral);
    let dot_nl = rotated.normal.dot(rotated.lateral);

    assert!(dot_dn.abs() < 1e-6);
    assert!(dot_dl.abs() < 1e-6);
    assert!(dot_nl.abs() < 1e-6);
}

#[test]
fn with_roll_changes_roll() {
    let frame = Frame::DEFAULT;
    let initial_roll = frame.roll();

    let rolled = frame.with_roll(PI / 4.0);
    let new_roll = rolled.roll();

    assert!((new_roll - (initial_roll + PI / 4.0)).abs() < 1e-6);
}

#[test]
fn chained_transformations() {
    let frame = Frame::DEFAULT;
    let transformed = frame
        .rotate_around(Float3::UP, PI / 2.0)
        .with_roll(PI / 4.0)
        .with_pitch(PI / 6.0);

    assert!((transformed.direction.magnitude() - 1.0).abs() < 1e-6);
    assert!((transformed.normal.magnitude() - 1.0).abs() < 1e-6);
    assert!((transformed.lateral.magnitude() - 1.0).abs() < 1e-6);
}
