import {
  AprilTagConfig,
  TagDisambiguationMode,
  TagNoiseAdjustMode,
  TagUseImuRotation,
} from "generated/thrift/gen-nodejs/pos_extrapolator_types";
import { reefscape_field } from "../tag_config/reefscape";
import { MatrixUtil, VectorUtil } from "../../util/math";
import { rebuilt_welded_field } from "../tag_config/rebuilt_welded";

const april_tag_pos_config: AprilTagConfig = {
  tag_position_config: rebuilt_welded_field,
  camera_position_config: {
    front_left: {
      position: VectorUtil.fromArray([0.38, 0.38, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(45),
    },
    front_right: {
      position: VectorUtil.fromArray([0.38, -0.38, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(-45),
    },
    rear_left: {
      position: VectorUtil.fromArray([-0.38, 0.38, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(135),
    },
    rear_right: {
      position: VectorUtil.fromArray([-0.38, -0.38, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(225),
    },
  },
  tag_use_imu_rotation: TagUseImuRotation.UNTIL_FIRST_NON_TAG_ROTATION,
  disambiguation_time_window_s: 0.05,
  tag_disambiguation_mode: TagDisambiguationMode.LEAST_ANGLE_AND_DISTANCE,
  tag_noise_adjust_mode: [TagNoiseAdjustMode.ADD_WEIGHT_PER_M_DISTANCE_TAG],
  tag_noise_adjust_config: {
    weight_per_m_from_distance_from_tag: 10,
    weight_per_degree_from_angle_error_tag: 0.05,
    weight_per_confidence_tag: 0.8,
  },
};

export default april_tag_pos_config;
