import { PathfindingConfig } from "generated/thrift/gen-nodejs/pathfinding_types";
import { fromImageToMap } from "../util/map";

const pathfinding_config: PathfindingConfig = {
  map_data: fromImageToMap("src/config/pathfinding/map.png"),
  lidar_config: {
    use_lidar: true,
    lidar_voxel_size_meters: 0.1,
    lidar_pub_topic: "lidar/lidar3d/pointcloud/3d/robotframe",
    unit_conversion: {
      non_unit_to_unit: 1,
      unit_to_non_unit: 1,
    },
  },
  others_config: {
    use_other_robot_positions: false,
    unit_conversion: {
      non_unit_to_unit: 1,
      unit_to_non_unit: 1,
    },
  },
  publish_map: true,
  map_pub_topic: "pathfinding_map",
  global_pose_pub_topic: "pos-extrapolator/robot-position",
  x_map_to_meters: {
    non_unit_to_unit: 5.698006, // px/m
    unit_to_non_unit: 0.1755, // m/px
  },
  y_map_to_meters: {
    non_unit_to_unit: 5.714286, // px/m
    unit_to_non_unit: 0.175, // m/px
  },
};

export default pathfinding_config;
