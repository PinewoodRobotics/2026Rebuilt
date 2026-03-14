import {
  KalmanFilterSensorType,
  type KalmanFilterConfig,
} from "generated/thrift/gen-nodejs/kalman_filter_types";
import { MatrixUtil, VectorUtil } from "../../util/math";

export const kalman_filter: KalmanFilterConfig = {
  initial_state_vector: VectorUtil.fromArray([
    14.437, 4.781, 0.0, 0.0, 0.0, 0.0,
  ]), // [x, y, vx, vy, angle, angular_velocity_rad_s]
  uncertainty_matrix: MatrixUtil.buildMatrixFromDiagonal([
    5.0, 5.0, 10.0, 10.0, 10.0, 1.0,
  ]),
  process_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
    0.0005, 0.0005, 1, 1, 0.01, 1,
  ]),
  sensors: {
    [KalmanFilterSensorType.APRIL_TAG]: {
      front_left: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          3.0, 3.0, 2.0,
        ]),
      },
      front_right: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          3.0, 3.0, 2.0,
        ]),
      },
      rear_left: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          3.0, 3.0, 2.0,
        ]),
      },
      rear_right: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          3.0, 3.0, 2.0,
        ]),
      },
    },
    [KalmanFilterSensorType.IMU]: {
      40: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          0.15, 0.15,
        ]),
      },
      41: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          0.15, 0.15,
        ]),
      },
    },
    [KalmanFilterSensorType.ODOMETRY]: {
      odom: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          5, 5, 0.1, 0.1,
        ]),
      },
    },
  },
};
