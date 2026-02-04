use common_core::proto::util::Vector3;

pub fn filter_points_rect(
    points: Vec<Vector3>,
    bottom_left: nalgebra::Vector3<f32>,
    top_right: nalgebra::Vector3<f32>,
) -> Vec<Vector3> {
    // `bottom_left` and `top_right` are treated as two opposite corners of an axis-aligned box.
    // They may not be ordered component-wise (e.g. bottom_left.y can be > top_right.y), so we
    // compute min/max bounds per axis.
    let min = nalgebra::Vector3::new(
        bottom_left.x.min(top_right.x),
        bottom_left.y.min(top_right.y),
        bottom_left.z.min(top_right.z),
    );
    let max = nalgebra::Vector3::new(
        bottom_left.x.max(top_right.x),
        bottom_left.y.max(top_right.y),
        bottom_left.z.max(top_right.z),
    );

    points
        .into_iter()
        .filter(|point| {
            point.x >= min.x
                && point.x <= max.x
                && point.y >= min.y
                && point.y <= max.y
                && point.z >= min.z
                && point.z <= max.z
        })
        .collect()
}

pub fn get_rect_corners(
    bottom_right_corner_world: nalgebra::Vector3<f32>,
    height: f32,
    width: f32,
    length: f32,
) -> (nalgebra::Vector3<f32>, nalgebra::Vector3<f32>) {
    let bottom_left_corner_world =
        bottom_right_corner_world + nalgebra::Vector3::new(0.0, width, 0.0);
    let top_right_corner_world =
        bottom_right_corner_world + nalgebra::Vector3::new(length, 0.0, height);
    (bottom_left_corner_world, top_right_corner_world)
}
