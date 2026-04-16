import {
  CameraParameters,
  CameraType,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../../util/math";
import { CameraConstants } from "../camera_constants";

const name = "front_right";

const front_right: CameraParameters = {
  pi_to_run_on: "tynan",
  name: name,
  camera_path: "/dev/usb_cam1",
  flags: 0,
  width: 800,
  height: 600,
  max_fps: 100,
  camera_matrix: MatrixUtil.buildMatrix([
    [454.851, 0.0, 407.49],
    [0.0, 454.6447, 336.0282],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.05002380001817732, -0.079, -0.0003529, -0.000122, 0.01493,
  ]),
  exposure_time: 30,
  camera_type: CameraType.OV2311,
  video_options: {
    send_feed: CameraConstants.kSendFeed,
    compression_quality: CameraConstants.kCompressionQuality,
    do_compression: true,
    publication_topic: "camera/" + name + "/video",
    overlay_tags: true,
  },
  do_detection: true,
};

export default front_right;
