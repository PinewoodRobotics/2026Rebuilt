import { MatrixUtil, VectorUtil } from "../../util/math";

export const nav_x_config = {
  "0": {
    use_position: false,
    use_rotation_absolute: true,
    use_rotation_velocity: true,
    use_velocity: false,
    imu_robot_position: {
      position: VectorUtil.fromArray([0.0, 0.0, 0.0]),
      rotation: MatrixUtil.buildMatrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
      ]),
    },
  },
  "1": {
    use_position: false,
    use_rotation_absolute: true,
    use_rotation_velocity: true,
    use_velocity: false,
    imu_robot_position: {
      position: VectorUtil.fromArray([0.0, 0.0, 0.0]),
      rotation: MatrixUtil.buildMatrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
      ]),
    },
  },
};
