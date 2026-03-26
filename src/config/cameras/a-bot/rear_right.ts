import {
  CameraType,
  type CameraParameters,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../../util/math";
import { CameraConstants } from "../camera_constants";

const name = "rear_right";

const rear_right: CameraParameters = {
  pi_to_run_on: "tynan",
  name: name,
  camera_path: "/dev/usb_cam4",
  flags: 0,
  width: 800,
  height: 600,
  max_fps: 100,
  camera_matrix: MatrixUtil.buildMatrix([
    [456.549, 0.0, 404.488],
    [0.0, 456.5886, 320.76],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.050907, -0.0819, -0.000034809075335798206, 0.0002923, 0.0186098,
  ]),
  exposure_time: 8,
  camera_type: CameraType.OV2311,
  video_options: {
    send_feed: CameraConstants.kSendFeed,
    compression_quality: CameraConstants.kCompressionQuality,
    overlay_tags: true,
    publication_topic: "camera/" + name + "/video",
    do_compression: true,
  },
  do_detection: true,
};

export default rear_right;
