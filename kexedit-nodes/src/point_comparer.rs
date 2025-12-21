use kexedit_core::Point;

use crate::golden::GoldPointData;

const BASE_TOLERANCE: f32 = 1e-3;
const TOLERANCE_PER_STEP: f32 = f32::EPSILON * 1024.0;

pub fn assert_points_match_gold(actual: &[Point], expected: &[GoldPointData]) {
    assert_eq!(
        expected.len(),
        actual.len(),
        "Point count mismatch: expected {}, got {}",
        expected.len(),
        actual.len()
    );

    let mut first_divergence = None;
    let mut max_drift = 0.0f32;
    let mut max_drift_index = 0;

    for (i, (act, exp)) in actual.iter().zip(expected.iter()).enumerate() {
        let drift = compute_drift(act, exp);
        if drift > max_drift {
            max_drift = drift;
            max_drift_index = i;
        }

        let tolerance = BASE_TOLERANCE + TOLERANCE_PER_STEP * i as f32;
        if first_divergence.is_none() && drift > tolerance {
            first_divergence = Some(i);
        }
    }

    eprintln!("=== DRIFT ANALYSIS ===");
    eprintln!("First divergence at index {:?}", first_divergence);
    eprintln!("Max drift {:e} at index {}", max_drift, max_drift_index);

    eprintln!("=== FIRST 20 POINTS ===");
    for i in 0..20.min(expected.len()) {
        log_point_comparison(&actual[i], &expected[i], i);
    }

    eprintln!("=== SAMPLE POINTS ===");
    for &i in &[50, 100, 200, 500, 1000, 1500] {
        if i < expected.len() {
            log_point_comparison(&actual[i], &expected[i], i);
        }
    }

    if let Some(div) = first_divergence {
        if div > 20 {
            eprintln!("=== AROUND FIRST DIVERGENCE ===");
            let start = div.saturating_sub(5);
            let end = (div + 6).min(expected.len());
            for i in start..end {
                log_point_comparison(&actual[i], &expected[i], i);
            }
        }
    }

    for i in get_sample_indices(expected.len()) {
        let tolerance = BASE_TOLERANCE + TOLERANCE_PER_STEP * i as f32;
        assert_point_matches_gold(&actual[i], &expected[i], i, tolerance);
    }
}

// Gold data uses legacy inverted naming: position = heart position
fn compute_drift(actual: &Point, expected: &GoldPointData) -> f32 {
    let mut max_drift = 0.0f32;
    max_drift = max_drift.max((actual.heart_position.x - expected.position.x).abs());
    max_drift = max_drift.max((actual.heart_position.y - expected.position.y).abs());
    max_drift = max_drift.max((actual.heart_position.z - expected.position.z).abs());
    max_drift = max_drift.max((actual.velocity - expected.velocity).abs());
    max_drift = max_drift.max((actual.energy - expected.energy).abs());
    max_drift
}

fn log_point_comparison(actual: &Point, expected: &GoldPointData, index: usize) {
    let tolerance = BASE_TOLERANCE + TOLERANCE_PER_STEP * index as f32;
    let marker = if compute_drift(actual, expected) > tolerance {
        ">>> "
    } else {
        "    "
    };

    let hp = actual.heart_position;
    eprintln!(
        "{}[{}] Pos: ({:.6}, {:.6}, {:.6}) vs ({:.6}, {:.6}, {:.6})",
        marker, index, hp.x, hp.y, hp.z, expected.position.x, expected.position.y, expected.position.z
    );
    eprintln!(
        "{}[{}] Vel: {:.6} vs {:.6}, diff={:e}",
        marker,
        index,
        actual.velocity,
        expected.velocity,
        (actual.velocity - expected.velocity).abs()
    );
    eprintln!(
        "{}[{}] Energy: {:.6} vs {:.6}, diff={:e}",
        marker,
        index,
        actual.energy,
        expected.energy,
        (actual.energy - expected.energy).abs()
    );
    eprintln!(
        "{}[{}] HeartArc: {:.6} vs {:.6}",
        marker,
        index,
        actual.heart_arc,
        expected.total_length
    );
}

fn get_sample_indices(count: usize) -> Vec<usize> {
    const BOUNDARY_COUNT: usize = 5;
    const SAMPLE_INTERVAL: usize = 50;

    let mut yielded = std::collections::HashSet::new();
    let mut indices = Vec::new();

    for i in 0..BOUNDARY_COUNT.min(count) {
        if yielded.insert(i) {
            indices.push(i);
        }
    }

    if count > BOUNDARY_COUNT {
        for i in count.saturating_sub(BOUNDARY_COUNT)..count {
            if yielded.insert(i) {
                indices.push(i);
            }
        }
    }

    let mut i = SAMPLE_INTERVAL;
    while i < count.saturating_sub(BOUNDARY_COUNT) {
        if yielded.insert(i) {
            indices.push(i);
        }
        i += SAMPLE_INTERVAL;
    }

    indices.sort_unstable();
    indices
}

// Gold data uses legacy inverted naming: position = heart position
fn assert_point_matches_gold(actual: &Point, expected: &GoldPointData, index: usize, tolerance: f32) {
    let hp = actual.heart_position;
    assert_float3(
        hp.x,
        hp.y,
        hp.z,
        expected.position.x,
        expected.position.y,
        expected.position.z,
        "HeartPosition",
        index,
        tolerance,
    );

    let dir = actual.direction;
    assert_float3(
        dir.x,
        dir.y,
        dir.z,
        expected.direction.x,
        expected.direction.y,
        expected.direction.z,
        "Direction",
        index,
        tolerance,
    );

    let lat = actual.lateral;
    assert_float3(
        lat.x,
        lat.y,
        lat.z,
        expected.lateral.x,
        expected.lateral.y,
        expected.lateral.z,
        "Lateral",
        index,
        tolerance,
    );

    let norm = actual.normal;
    assert_float3(
        norm.x,
        norm.y,
        norm.z,
        expected.normal.x,
        expected.normal.y,
        expected.normal.z,
        "Normal",
        index,
        tolerance,
    );

    assert_float(actual.velocity, expected.velocity, "Velocity", index, tolerance);
    assert_float(actual.energy, expected.energy, "Energy", index, tolerance);

    assert_float(
        actual.normal_force,
        expected.normal_force,
        "NormalForce",
        index,
        tolerance,
    );
    assert_float(
        actual.lateral_force,
        expected.lateral_force,
        "LateralForce",
        index,
        tolerance,
    );

    assert_float(actual.heart_arc, expected.total_length, "HeartArc", index, tolerance);
    assert_float(
        actual.spine_arc,
        expected.total_heart_length,
        "SpineArc",
        index,
        tolerance,
    );

    assert_float(actual.roll_speed, expected.roll_speed, "RollSpeed", index, tolerance);
    assert_float(
        actual.heart_offset,
        expected.heart,
        "HeartOffset",
        index,
        tolerance,
    );
    assert_float(actual.friction, expected.friction, "Friction", index, tolerance);
    assert_float(actual.resistance, expected.resistance, "Resistance", index, tolerance);
}

fn assert_float(actual: f32, expected: f32, field: &str, index: usize, tolerance: f32) {
    let diff = (expected - actual).abs();
    assert!(
        diff <= tolerance,
        "Point[{}].{}: expected {:e}, got {:e}, diff {:e} (tolerance {:e})",
        index,
        field,
        expected,
        actual,
        diff,
        tolerance
    );
}

fn assert_float3(
    ax: f32,
    ay: f32,
    az: f32,
    ex: f32,
    ey: f32,
    ez: f32,
    field: &str,
    index: usize,
    tolerance: f32,
) {
    assert_float(ax, ex, &format!("{}.x", field), index, tolerance);
    assert_float(ay, ey, &format!("{}.y", field), index, tolerance);
    assert_float(az, ez, &format!("{}.z", field), index, tolerance);
}
