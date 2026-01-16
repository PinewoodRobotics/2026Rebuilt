import {
  CameraType,
  type CameraParameters,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../util/math";

const rear_right: CameraParameters = {
  pi_to_run_on: "agathaking",
  name: "rear_right",
  camera_path: "/dev/usb_top_left_cam",
  flags: 0,
  width: 800,
  height: 600,
  max_fps: 100,
  camera_matrix: MatrixUtil.buildMatrix([
    [450.2109835443841, 0.0, 408.7319259851182],
    [0.0, 450.2304845946486, 322.1238106252265],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.0515984440654759, -0.09297623176342812, 0.0002654901359653783,
    0.00027415514880627453, 0.029874953478503052,
  ]),
  exposure_time: 10,
  camera_type: 0 as CameraType,
  video_options: {
    send_feed: false,
    overlay_tags: false,
  },
};

export default rear_right;
