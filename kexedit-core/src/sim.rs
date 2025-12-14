use std::f32::consts::PI;

pub const G: f32 = 9.80665;
pub const HZ: f32 = 100.0;
pub const DT: f32 = 1.0 / HZ;
pub const EPSILON: f32 = 1.192_093e-7;
pub const MIN_VELOCITY: f32 = 1e-3;

pub fn wrap_angle(rad: f32) -> f32 {
    if (-PI..=PI).contains(&rad) {
        return rad;
    }
    const TWO_PI: f32 = 2.0 * PI;
    const THREE_PI: f32 = 3.0 * PI;
    (rad + THREE_PI) % TWO_PI - PI
}

pub fn compute_total_energy(
    velocity: f32,
    center_y: f32,
    friction_distance: f32,
    friction: f32,
) -> f32 {
    0.5 * velocity * velocity + G * center_y + G * friction_distance * friction
}

pub fn update_energy(
    prev_energy: f32,
    prev_velocity: f32,
    center_y: f32,
    friction_distance: f32,
    friction: f32,
    resistance: f32,
) -> (f32, f32) {
    let pe = G * (center_y + friction_distance * friction);
    let new_energy = prev_energy - prev_velocity * prev_velocity * prev_velocity * resistance * DT;
    let new_velocity = (2.0 * (new_energy - pe).max(0.0)).sqrt();
    (new_energy, new_velocity)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    const TOLERANCE: f32 = 1e-4;

    #[test]
    fn constants_gravity_matches_standard() {
        assert_relative_eq!(G, 9.80665, epsilon = TOLERANCE);
    }

    #[test]
    fn constants_simulation_rate() {
        assert_relative_eq!(HZ, 100.0, epsilon = TOLERANCE);
    }

    #[test]
    fn dt_equals_one_over_hz() {
        assert_relative_eq!(DT, 1.0 / HZ, epsilon = TOLERANCE);
        assert_relative_eq!(DT, 0.01, epsilon = TOLERANCE);
    }

    #[test]
    fn constants_min_velocity() {
        assert_relative_eq!(MIN_VELOCITY, 1e-3, epsilon = TOLERANCE);
    }

    #[test]
    fn wrap_angle_in_range_unchanged() {
        let angles = [0.0, 0.5, -0.5, PI - 0.1, -PI + 0.1];
        for angle in angles {
            let wrapped = wrap_angle(angle);
            assert_relative_eq!(wrapped, angle, epsilon = TOLERANCE);
        }
    }

    #[test]
    fn wrap_angle_greater_than_pi_wraps_negative() {
        let angle = PI + 0.5;
        let wrapped = wrap_angle(angle);

        assert!(wrapped > -PI);
        assert!(wrapped <= PI);
        assert!(wrapped < 0.0);
    }

    #[test]
    fn wrap_angle_less_than_negative_pi_wraps_positive() {
        let angle = -PI - 0.5;
        let wrapped = wrap_angle(angle);

        assert!(wrapped > -PI);
        assert!(wrapped <= PI);
        assert!(wrapped > 0.0);
    }

    #[test]
    fn wrap_angle_exactly_pi_returns_valid_range() {
        let wrapped = wrap_angle(PI);
        assert!(wrapped >= -PI);
        assert!(wrapped <= PI);
    }

    #[test]
    fn wrap_angle_large_positive_wraps_correctly() {
        let angle = 2.0 * PI + 0.3;
        let wrapped = wrap_angle(angle);

        assert!(wrapped > -PI);
        assert!(wrapped <= PI);
        assert_relative_eq!(wrapped, 0.3, epsilon = TOLERANCE);
    }

    #[test]
    fn wrap_angle_moderate_negative_wraps_correctly() {
        let angle = -2.0 * PI - 0.3;
        let wrapped = wrap_angle(angle);

        assert!(wrapped > -PI);
        assert!(wrapped <= PI);
    }

    #[test]
    fn compute_total_energy_at_rest_only_potential() {
        let velocity = 0.0;
        let center_y = 10.0;
        let friction_distance = 0.0;
        let friction = 0.0;

        let energy = compute_total_energy(velocity, center_y, friction_distance, friction);
        let expected_pe = G * center_y;

        assert_relative_eq!(energy, expected_pe, epsilon = TOLERANCE);
    }

    #[test]
    fn compute_total_energy_moving_includes_kinetic() {
        let velocity = 10.0;
        let center_y = 0.0;
        let friction_distance = 0.0;
        let friction = 0.0;

        let energy = compute_total_energy(velocity, center_y, friction_distance, friction);
        let expected_ke = 0.5 * velocity * velocity;

        assert_relative_eq!(energy, expected_ke, epsilon = TOLERANCE);
    }

    #[test]
    fn compute_total_energy_with_friction_adds_friction_term() {
        let velocity = 10.0;
        let center_y = 5.0;
        let friction_distance = 100.0;
        let friction = 0.02;

        let energy = compute_total_energy(velocity, center_y, friction_distance, friction);
        let expected_ke = 0.5 * velocity * velocity;
        let expected_pe = G * center_y;
        let expected_friction = G * friction_distance * friction;

        assert_relative_eq!(
            energy,
            expected_ke + expected_pe + expected_friction,
            epsilon = TOLERANCE
        );
    }

    #[test]
    fn update_energy_with_resistance_energy_decreases() {
        let prev_energy = 500.0;
        let prev_velocity = 20.0;
        let center_y = 0.0;
        let friction_distance = 0.0;
        let friction = 0.0;
        let resistance = 0.001;

        let (new_energy, _) =
            update_energy(prev_energy, prev_velocity, center_y, friction_distance, friction, resistance);

        assert!(new_energy < prev_energy);

        let (_, velocity_no_resistance) =
            update_energy(prev_energy, prev_velocity, center_y, friction_distance, friction, 0.0);
        let (_, velocity_with_resistance) =
            update_energy(prev_energy, prev_velocity, center_y, friction_distance, friction, resistance);

        assert!(velocity_with_resistance < velocity_no_resistance);
    }

    #[test]
    fn update_energy_zero_resistance_conserves_energy() {
        let prev_energy = 500.0;
        let prev_velocity = 20.0;
        let center_y = 0.0;
        let friction_distance = 0.0;
        let friction = 0.0;
        let resistance = 0.0;

        let (new_energy, _) =
            update_energy(prev_energy, prev_velocity, center_y, friction_distance, friction, resistance);

        assert_relative_eq!(new_energy, prev_energy, epsilon = TOLERANCE);
    }

    #[test]
    fn update_energy_higher_altitude_lower_velocity() {
        let energy = 500.0;
        let velocity = 20.0;
        let low_center_y = 0.0;
        let high_center_y = 10.0;
        let resistance = 0.0;

        let (_, velocity_low) = update_energy(energy, velocity, low_center_y, 0.0, 0.0, resistance);
        let (_, velocity_high) = update_energy(energy, velocity, high_center_y, 0.0, 0.0, resistance);

        assert!(velocity_low > velocity_high);
    }

    #[test]
    fn update_energy_insufficient_energy_velocity_becomes_zero() {
        let prev_energy = 50.0;
        let prev_velocity = 5.0;
        let center_y = 100.0;
        let friction_distance = 0.0;
        let friction = 0.0;
        let resistance = 0.0;

        let (_, new_velocity) =
            update_energy(prev_energy, prev_velocity, center_y, friction_distance, friction, resistance);

        assert_relative_eq!(new_velocity, 0.0, epsilon = TOLERANCE);
    }

    #[test]
    fn update_energy_velocity_cubed_resistance() {
        let prev_energy = 1000.0;
        let slow_velocity = 5.0;
        let fast_velocity = 20.0;
        let resistance = 0.001;

        let (slow_energy, _) = update_energy(prev_energy, slow_velocity, 0.0, 0.0, 0.0, resistance);
        let (fast_energy, _) = update_energy(prev_energy, fast_velocity, 0.0, 0.0, 0.0, resistance);

        let slow_loss = prev_energy - slow_energy;
        let fast_loss = prev_energy - fast_energy;

        assert!(fast_loss > slow_loss);
        let expected_ratio = (fast_velocity * fast_velocity * fast_velocity)
            / (slow_velocity * slow_velocity * slow_velocity);
        let actual_ratio = fast_loss / slow_loss;
        assert_relative_eq!(actual_ratio, expected_ratio, epsilon = 2.0);
    }
}
