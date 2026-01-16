import {
  CameraType,
  type CameraParameters,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../util/math";

const rear_left: CameraParameters = {
  pi_to_run_on: "agathaking",
  name: "rear_left",
  camera_path: "/dev/usb_bottom_left_cam",
  flags: 0,
  width: 800,
  height: 600,
  max_fps: 100,
  camera_matrix: MatrixUtil.buildMatrix([
    [453.9992399510156, 0.0, 399.7655704599877],
    [0.0, 453.8751071333631, 317.17013783279384],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.04825041839384538, -0.08704195897420881, 0.000026853384564275713,
    0.00031916717015169235, 0.027065688766617867,
  ]),
  exposure_time: 150,
  camera_type: CameraType.ULTRAWIDE_100,
  brightness: 200,
  video_options: {
    send_feed: true,
    overlay_tags: true,
    publication_topic: "camera/rear_left/video",
    compression_quality: 30,
    do_compression: true,
  },
};

export default rear_left;
