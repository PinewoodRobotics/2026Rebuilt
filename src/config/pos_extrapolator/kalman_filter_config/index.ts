import {
  KalmanFilterSensorType,
  type KalmanFilterConfig,
} from "generated/thrift/gen-nodejs/kalman_filter_types";
import { MatrixUtil, VectorUtil } from "../../util/math";

export const kalman_filter: KalmanFilterConfig = {
  state_vector: VectorUtil.fromArray([2.0, 5.0, 0.0, 0.0, 1.0, 0.0]), // [x, y, vx, vy, angle, angular_velocity_rad_s]
  uncertainty_matrix: MatrixUtil.buildMatrixFromDiagonal([
    5.0, 5.0, 0.01, 10.0, 10.0, 5.0,
  ]),
  process_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
    0.01, 0.01, 1, 1, 1, 1,
  ]),
  dim_x_z: [6, 6],
  sensors: {
    [KalmanFilterSensorType.APRIL_TAG]: {
      front_left: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          3.0, 3.0, 5.0,
        ]),
      },
      front_right: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          3.0, 3.0, 5.0,
        ]),
      },
      rear_left: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          3.0, 3.0, 5.0,
        ]),
      },
      rear_right: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          3.0, 3.0, 5.0,
        ]),
      },
    },
    [KalmanFilterSensorType.IMU]: {
      0: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([0, 0]),
      },
      1: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([0, 0]),
      },
    },
    [KalmanFilterSensorType.ODOMETRY]: {
      odom: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          5, 5, 0, 0,
        ]),
      },
    },
  },
};
