import {
  AprilTagConfig,
  TagNoiseAdjustMode,
} from "generated/thrift/gen-nodejs/pos_extrapolator_types";
import { MatrixUtil, VectorUtil } from "../../util/math";
import { rebuilt_welded_field } from "../tag_config/rebuilt_welded";

const april_tag_pos_config: AprilTagConfig = {
  tag_position_config: rebuilt_welded_field,
  camera_position_config: {
    front_left: {
      position: VectorUtil.fromArray([0.0813, 0.324, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(45),
    },
    front_right: {
      position: VectorUtil.fromArray([0.136, -0.324, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(-52.904),
    },
    rear_left: {
      position: VectorUtil.fromArray([-0.0813, 0.324, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(135),
    },
    rear_right: {
      position: VectorUtil.fromArray([-0.297, -0.297, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(225),
    },
  },
  noise_change_modes: [
    // TagNoiseAdjustMode.ADD_WEIGHT_PER_M_DISTANCE_TAG,
    TagNoiseAdjustMode.ADD_WEIGHT_PER_TAG_CONFIDENCE,
  ],
  tag_noise_adjust_config: {
    weight_per_m_from_distance_from_tag: 0.3,
    weight_per_degree_from_angle_error_tag: 0.0,
    weight_per_confidence_tag: 0.04,
    min_distance_from_tag_to_use_noise_adjustment: 1.5,
  },
  insert_predicted_global_rotation: true,
  apriltag_mahalanobis_gate_threshold: 5.0,
};

export default april_tag_pos_config;
