import { ImuConfig } from "generated/thrift/gen-nodejs/pos_extrapolator_types";
import { MatrixUtil, VectorUtil } from "../../util/math";

export const nav_x_config: { [k: string]: ImuConfig } = {
  "41": {
    use_position: false,
    use_rotation: true,
    use_velocity: false,
  },
  "40": {
    use_position: false,
    use_rotation: true,
    use_velocity: false,
  },
};
