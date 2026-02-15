import { ImuConfig } from "generated/thrift/gen-nodejs/pos_extrapolator_types";
import { MatrixUtil, VectorUtil } from "../../util/math";

export const nav_x_config: { [k: string]: ImuConfig } = {
  "0": {
    use_position: false,
    use_rotation: true,
    use_velocity: false,
  },
  "1": {
    use_position: false,
    use_rotation: true,
    use_velocity: false,
  },
};
