import {
  CameraParameters,
  CameraType,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../util/math";

const front_right: CameraParameters = {
  pi_to_run_on: "tynan",
  name: "front_right",
  camera_path: "/dev/usb_cam2",
  flags: 0,
  width: 800,
  height: 600,
  max_fps: 100,
  camera_matrix: MatrixUtil.buildMatrix([
    [685.1818937239501, 0.0, 414.2897986078692],
    [0.0, 685.3366953090806, 307.27153822775506],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.03841168751857183, -0.03836398867353221, 0.00011911539228647425,
    -0.00047135490659979865, -0.011145047269650642,
  ]),
  exposure_time: 10,
  camera_type: CameraType.OV2311,
  video_options: {
    send_feed: false,
    do_compression: true,
    overlay_tags: false,
    compression_quality: 30,
  },
};

export default front_right;
