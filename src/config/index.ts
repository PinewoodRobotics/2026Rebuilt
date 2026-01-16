import type { Config } from "generated/thrift/gen-nodejs/config_types";
import { april_tag_detection_config } from "./april_tags_detection";
import prod1 from "./cameras/prod_1";
import lidar_configs from "./lidar";
import pathfinding_config from "./pathfinding";
import { pose_extrapolator } from "./pos_extrapolator";
import front_left from "./cameras/front_left";
import front_right from "./cameras/front_right";
import rear_left from "./cameras/rear_left";
import rear_right from "./cameras/rear_right";
import jetson_cam from "./cameras/jetson_cam";
import logitech_cam from "./cameras/logitech_cam";

const config: Config = {
  pos_extrapolator: pose_extrapolator,
  cameras: [front_left, front_right, rear_left, rear_right],
  april_detection: april_tag_detection_config,
  lidar_configs: lidar_configs,
  pathfinding: pathfinding_config,
  record_replay: false,
  replay_folder_path: "replays",
  object_recognition: {
    cameras_to_use: [],
    model_path: "",
    output_topic: "",
    objects_to_detect: [],
    device: "",
    iou_threshold: 0,
    conf_threshold: 0,
  },
};

export default config;
