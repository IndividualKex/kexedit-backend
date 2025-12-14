use kexedit_core::Point;

pub fn build(path: &[Point]) -> Vec<Point> {
    if path.is_empty() {
        return Vec::new();
    }

    path.iter()
        .rev()
        .map(|p| {
            Point::new(
                p.spine_position,
                -p.direction,
                p.normal,
                -p.lateral,
                p.velocity,
                p.energy,
                p.normal_force,
                -p.lateral_force,
                p.heart_arc,
                p.spine_arc,
                p.spine_advance,
                p.friction_origin,
                p.roll_speed,
                p.heart_offset,
                p.friction,
                p.resistance,
            )
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_empty_path_returns_empty() {
        let result = build(&[]);
        assert_eq!(result.len(), 0);
    }

    #[test]
    fn build_reverses_path_order() {
        let path = vec![
            Point::DEFAULT,
            Point::DEFAULT.with_friction_origin(1.0),
            Point::DEFAULT.with_friction_origin(2.0),
        ];

        let result = build(&path);

        assert_eq!(result.len(), 3);
        assert_eq!(result[0].friction_origin, 2.0);
        assert_eq!(result[1].friction_origin, 1.0);
        assert_eq!(result[2].friction_origin, 0.0);
    }

    #[test]
    fn build_reverses_direction_for_each_point() {
        let path = vec![Point::DEFAULT];
        let result = build(&path);

        assert_eq!(result[0].direction, -path[0].direction);
        assert_eq!(result[0].lateral, -path[0].lateral);
    }
}
