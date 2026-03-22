import {
  KalmanFilterSensorType,
  type KalmanFilterConfig,
} from "generated/thrift/gen-nodejs/kalman_filter_types";
import { MatrixUtil, VectorUtil } from "../../util/math";

const tag_noise_default_xy = 0.0; // 1
const tag_noise_default_theta = 0.0; // 2 * Math.PI;

export const kalman_filter: KalmanFilterConfig = {
  initial_state_vector: VectorUtil.fromArray([14.437, 4.781, 0.0]), // [x, y, theta]
  uncertainty_matrix: MatrixUtil.buildMatrixFromDiagonal([0.0, 0.0, 0.0]),
  process_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
    0.125 / 2,
    0.125 / 2,
    Math.PI / 16,
  ]),
  sensors: {
    [KalmanFilterSensorType.APRIL_TAG]: {
      front_left: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          tag_noise_default_xy,
          tag_noise_default_xy,
          tag_noise_default_theta,
        ]),
      },
      front_right: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          tag_noise_default_xy,
          tag_noise_default_xy,
          tag_noise_default_theta,
        ]),
      },
      rear_left: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          tag_noise_default_xy,
          tag_noise_default_xy,
          tag_noise_default_theta,
        ]),
      },
      rear_right: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          tag_noise_default_xy,
          tag_noise_default_xy,
          tag_noise_default_theta,
        ]),
      },
    },
  },
};
