import {
  CameraParameters,
  CameraType,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../../util/math";

const front_right: CameraParameters = {
  pi_to_run_on: "tynan",
  name: "front_right",
  camera_path: "/dev/usb_cam3",
  flags: 0,
  width: 800,
  height: 600,
  max_fps: 100,
  camera_matrix: MatrixUtil.buildMatrix([
    [456.4566091243219, 0.0, 403.4510675692207],
    [0.0, 456.4929611660038, 320.254620681183],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.05147776259797679, -0.08376762888571426, -0.0005087791220038304,
    -5.9848176245483235e-5, 0.020125747371733234,
  ]),
  exposure_time: 10,
  camera_type: CameraType.OV2311,
  video_options: {
    send_feed: false,
    do_compression: true,
    overlay_tags: true,
    compression_quality: 10,
  },
  do_detection: true,
};

export default front_right;
