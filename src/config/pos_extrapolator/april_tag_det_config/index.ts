import {
  AprilTagConfig,
  TagDisambiguationMode,
  TagUseImuRotation,
} from "generated/thrift/gen-nodejs/pos_extrapolator_types";
import { reefscape_field } from "../tag_config/reefscape";
import { MatrixUtil, VectorUtil } from "../../util/math";

const april_tag_pos_config: AprilTagConfig = {
  tag_position_config: reefscape_field,
  camera_position_config: {
    front_left: {
      position: VectorUtil.fromArray([0.33, 0.33, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(45),
    },
    front_right: {
      position: VectorUtil.fromArray([0.33, -0.33, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(-45),
    },
    rear_left: {
      position: VectorUtil.fromArray([-0.33, 0.33, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(135),
    },
    rear_right: {
      position: VectorUtil.fromArray([-0.33, -0.33, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(225),
    },
  },
  tag_use_imu_rotation: TagUseImuRotation.NEVER,
  disambiguation_time_window_s: 0.05,
  tag_disambiguation_mode: TagDisambiguationMode.LEAST_ANGLE_AND_DISTANCE,
};

export default april_tag_pos_config;
