import {
  CameraType,
  type CameraParameters,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../util/math";

const rear_right: CameraParameters = {
  pi_to_run_on: "nathan-hale",
  name: "rear_right",
  camera_path: "/dev/usb_cam3",
  flags: 0,
  width: 800,
  height: 600,
  max_fps: 100,
  camera_matrix: MatrixUtil.buildMatrix([
    // front left camera calib
    [685.088010528533, 0.0, 400.456913426101],
    [0.0, 685.286311116306, 283.611674040712],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.0370949377097917, 0.041319913100527, -0.00128168607814328,
    -0.00128042296949747, -0.298591139932664,
  ]),
  exposure_time: 10,
  camera_type: CameraType.OV2311,
  video_options: {
    send_feed: false,
    overlay_tags: false,
  },
};

export default rear_right;
