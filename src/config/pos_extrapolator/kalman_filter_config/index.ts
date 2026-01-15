import {
  KalmanFilterSensorType,
  type KalmanFilterConfig,
} from "generated/thrift/gen-nodejs/kalman_filter_types";
import { MatrixUtil, VectorUtil } from "../../util/math";

export const kalman_filter: KalmanFilterConfig = {
  state_vector: VectorUtil.fromArray([2.0, 5.0, 0.0, 0.0, 1.0, 0.0, 0.0]), // [x, y, vx, vy, cos, sin, angular_velocity_rad_s]
  time_step_initial: 0.1,
  state_transition_matrix: MatrixUtil.buildMatrixFromDiagonal([
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
  ]),
  uncertainty_matrix: MatrixUtil.buildMatrixFromDiagonal([
    10.0, 10.0, 2.0, 2.0, 1.0, 1.0, 1.0,
  ]),
  process_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
    0.001, 0.001, 0.1, 0.1, 0.001, 0.001, 0.001,
  ]),
  dim_x_z: [7, 7],
  sensors: {
    [KalmanFilterSensorType.APRIL_TAG]: {
      front_left: {
        measurement_conversion_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1, 1, 1, 1, 1, 1,
        ]),
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1.0, 1.0, 5.0, 5.0,
        ]),
      },
      front_right: {
        measurement_conversion_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1, 1, 1, 1, 1, 1,
        ]),
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1.0, 1.0, 5.0, 5.0,
        ]),
      },
      rear_left: {
        measurement_conversion_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1, 1, 1, 1, 1, 1,
        ]),
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1.0, 1.0, 5.0, 5.0,
        ]),
      },
      rear_right: {
        measurement_conversion_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1, 1, 1, 1, 1, 1,
        ]),
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1.0, 1.0, 5.0, 5.0,
        ]),
      },
    },
    [KalmanFilterSensorType.IMU]: {
      0: {
        measurement_conversion_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1, 1, 1, 1, 1, 1,
        ]),
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          0.01, 0.01, 0.001,
        ]),
      },
      1: {
        measurement_conversion_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1, 1, 1, 1, 1, 1,
        ]),
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          0.01, 0.01, 0.001,
        ]),
      },
    },
    [KalmanFilterSensorType.ODOMETRY]: {
      odom: {
        measurement_conversion_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        ]),
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          5, 5, 0.01, 0.01,
        ]), // 5, 5,
      },
    },
  },
};
