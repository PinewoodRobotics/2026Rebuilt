use common_core::proto::util::Vector3;

pub fn filter_points_rect(
    points: Vec<Vector3>,
    bottom_left: nalgebra::Vector3<f32>,
    top_right: nalgebra::Vector3<f32>,
) -> Vec<Vector3> {
    points
        .into_iter()
        .filter(|point| {
            point.x >= bottom_left.x
                && point.x <= top_right.x
                && point.y >= bottom_left.y
                && point.y <= top_right.y
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
