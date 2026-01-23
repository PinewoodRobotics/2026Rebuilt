use autobahn_client::autobahn::{Address, Autobahn};
use common_core::proto::{
    sensor::{
        general_sensor_data::Data, lidar_data, GeneralSensorData, LidarData, PointCloud3d,
        SensorName,
    },
    util::Vector3,
};
use futures_util::StreamExt;
use prost::Message;
use unitree_lidar_l1_rust::lidar::reader::{LidarReader, LidarResult};

mod math;

const AUTOBAHN_HOST: &str = "localhost";
const AUTOBAHN_PORT: u16 = 9000;

const CLOUD_SCAN_NUM: u32 = 28;
const PORT: &str = "/dev/ttyUSB0";
const BAUD_RATE: u32 = 2000000;
const MIN_DISTANCE_METERS: f64 = 0.0;
const MAX_DISTANCE_METERS: f64 = 40.0;

const LIDAR_NAME: &str = "lidar-1";

const BOTTOM_RIGHT_CORNER_WORLD: nalgebra::Vector3<f32> = nalgebra::Vector3::new(0.0, 0.0, 0.0);
const HEIGHT: f32 = 5.0;
const WIDTH: f32 = 10.0;
const LENGTH: f32 = 10.0;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let autobahn = Autobahn::new_default(Address::new(AUTOBAHN_HOST.to_string(), AUTOBAHN_PORT));
    autobahn.begin().await?;

    let mut reader = LidarReader::new_with_initialize(
        CLOUD_SCAN_NUM,
        PORT.to_string(),
        BAUD_RATE,
        MIN_DISTANCE_METERS.into(),
        MAX_DISTANCE_METERS.into(),
        0.0,
        0.001,
        0.0,
    )?;
    reader.start_lidar()?;

    let (bottom_left_corner_world, top_right_corner_world) =
        math::get_rect_corners(BOTTOM_RIGHT_CORNER_WORLD, HEIGHT, WIDTH, LENGTH);

    let mut reader = reader.into_stream();
    while let Some(result) = reader.next().await {
        match result {
            LidarResult::PointCloud(points) => {
                let points = points
                    .iter()
                    .map(|point| Vector3 {
                        x: point.x,
                        y: point.y,
                        z: point.z,
                    })
                    .collect::<Vec<_>>();
                let points = math::filter_points_rect(
                    points,
                    bottom_left_corner_world,
                    top_right_corner_world,
                );

                let general_sensor_data = GeneralSensorData {
                    sensor_name: SensorName::Lidar as i32,
                    sensor_id: LIDAR_NAME.to_string(),
                    timestamp: std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap()
                        .as_secs() as i64,
                    data: Some(Data::Lidar(LidarData {
                        data: Some(lidar_data::Data::PointCloud3d(PointCloud3d {
                            ranges: points,
                            lidar_id: LIDAR_NAME.to_string(),
                        })),
                    })),
                    processing_time_ms: 0,
                };

                let _ = autobahn
                    .publish(
                        &format!("lidar/lidar3d/pointcloud"),
                        general_sensor_data.encode_to_vec(),
                    )
                    .await;
            }
            LidarResult::ImuReading(imu) => {}
        }
    }

    Ok(())
}
