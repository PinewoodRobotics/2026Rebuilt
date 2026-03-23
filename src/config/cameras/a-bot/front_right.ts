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
    [455.70786440844495, 0.0, 406.91175040857706],
    [0.0, 455.64862622851547, 335.62047654299613],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.05002380001817732, -0.08370114548224765, -0.0005494029888528023,
    4.610242169757393e-6, 0.02207457164458876,
  ]),
  exposure_time: 8,
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
