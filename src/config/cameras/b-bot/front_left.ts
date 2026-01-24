import {
  CameraParameters,
  CameraType,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../../util/math";

const front_left: CameraParameters = {
  pi_to_run_on: "agatha-king",
  name: "front_left",
  camera_path: "/dev/usb_cam1",
  flags: 0,
  width: 800,
  height: 600,
  max_fps: 100,
  camera_matrix: MatrixUtil.buildMatrix([
    [456.81085799059014, 0.0, 395.06984136844125],
    [0.0, 456.7987342006777, 334.73571738807914],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.052745830893280964, -0.08619099299637119, -0.00044316972116126193,
    0.00004422164981020342, 0.022592221476200467,
  ]),
  exposure_time: 10,
  camera_type: CameraType.OV2311,
  video_options: {
    send_feed: false,
    compression_quality: 10,
    publication_topic: "camera/front_left/video",
    do_compression: true,
    overlay_tags: true,
  },
  do_detection: true,
};

export default front_left;
