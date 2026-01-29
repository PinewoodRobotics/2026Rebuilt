import {
  OdomConfig,
  OdometryPositionSource,
} from "generated/thrift/gen-nodejs/pos_extrapolator_types";
import { MatrixUtil, VectorUtil } from "../../util/math";

export const swerve_odom_config: OdomConfig = {
  position_source: OdometryPositionSource.DONT_USE,
  use_rotation: false,
};
