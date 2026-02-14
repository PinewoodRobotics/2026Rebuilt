import {
  KalmanFilterSensorType,
  type KalmanFilterConfig,
} from "generated/thrift/gen-nodejs/kalman_filter_types";
import { MatrixUtil, VectorUtil } from "../../util/math";

export const kalman_filter: KalmanFilterConfig = {
  dim_x_z: [7, 7],
  state_vector: VectorUtil.fromArray([2.0, 5.0, 0.0, 0.0, 1.0, 0.0, 0.0]), // [x, y, vx, vy, cos, sin, angular_velocity_rad_s]
  uncertainty_matrix: MatrixUtil.buildMatrixFromDiagonal([
    5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
  ]),
  process_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
    0.0001,
    0.0001,
    1,
    1,
    1,
    1,
    1, // lower is worse BTW. The higher the number, the more the filter follows the measurements instead of the model (predict step)
  ]),
  sensors: {
    [KalmanFilterSensorType.APRIL_TAG]: {
      front_left: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1.0, 1.0, 5.0, 5.0,
        ]),
      },
      front_right: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1.0, 1.0, 5.0, 5.0,
        ]),
      },
      rear_left: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1.0, 1.0, 5.0, 5.0,
        ]),
      },
      rear_right: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          1.0, 1.0, 5.0, 5.0,
        ]),
      },
    },
    [KalmanFilterSensorType.IMU]: {
      0: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          0.01, 0.01, 0.001,
        ]),
      },
      1: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          0.01, 0.01, 0.001,
        ]),
      },
    },
    [KalmanFilterSensorType.ODOMETRY]: {
      odom: {
        measurement_noise_matrix: MatrixUtil.buildMatrixFromDiagonal([
          0.001, 0.001,
        ]),
      },
    },
  },
};
