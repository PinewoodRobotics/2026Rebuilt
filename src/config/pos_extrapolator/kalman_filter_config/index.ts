import {
  KalmanFilterSensorType,
  type KalmanFilterConfig,
} from "generated/thrift/gen-nodejs/kalman_filter_types";
import { MatrixUtil, VectorUtil } from "../../util/math";

export const kalman_filter: KalmanFilterConfig = {
  initial_state_vector: VectorUtil.fromArray([14.437, 4.781, 0.0]), // [x, y, theta]
  uncertainty_matrix: MatrixUtil.buildMatrixFromDiagonal([5.0, 5.0, 10.0]),
  process_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([1, 1, 1]),
  sensors: {
    [KalmanFilterSensorType.APRIL_TAG]: {
      front_left: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          3.0, 3.0, 10.0,
        ]),
      },
      front_right: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          3.0, 3.0, 10.0,
        ]),
      },
      rear_left: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          3.0, 3.0, 10.0,
        ]),
      },
      rear_right: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          3.0, 3.0, 10.0,
        ]),
      },
    },
  },
};
