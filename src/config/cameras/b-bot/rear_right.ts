import {
  CameraType,
  type CameraParameters,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../../util/math";

const rear_right: CameraParameters = {
  pi_to_run_on: "nathan-hale",
  name: "rear_right",
  camera_path: "/dev/usb_cam1",
  flags: 0,
  width: 800,
  height: 600,
  max_fps: 100,
  camera_matrix: MatrixUtil.buildMatrix([
    [456.10504968438045, 0.0, 403.6933383290121],
    [0.0, 456.0604482158868, 341.09074241391494],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.04841029488157198, -0.08174454831935413, 0.0001501040390929917,
    0.00011501008144279749, 0.021698542194869413,
  ]),
  exposure_time: 10,
  camera_type: CameraType.OV2311,
  video_options: {
    send_feed: false,
    overlay_tags: true,
    publication_topic: "camera/rear_right/video",
    compression_quality: 10,
    do_compression: true,
  },
  do_detection: true,
};

export default rear_right;
