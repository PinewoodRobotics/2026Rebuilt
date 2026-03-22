import {
  CameraParameters,
  CameraType,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../../util/math";
import { CameraConstants } from "../camera_constants";

const name = "rear_left";

const rear_left: CameraParameters = {
  pi_to_run_on: "agatha-king",
  name: name,
  camera_path: "/dev/usb_cam3",
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
  exposure_time: 8,
  camera_type: CameraType.OV2311,
  video_options: {
    send_feed: CameraConstants.kSendFeed,
    compression_quality: CameraConstants.kCompressionQuality,
    publication_topic: "camera/" + name + "/video",
    do_compression: true,
    overlay_tags: true,
  },
  do_detection: true,
};

export default rear_left;
