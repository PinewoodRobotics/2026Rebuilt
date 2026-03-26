import {
  CameraType,
  type CameraParameters,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../../util/math";
import { CameraConstants } from "../camera_constants";

const name = "front_left";

const front_left: CameraParameters = {
  pi_to_run_on: "agatha-king",
  name: name,
  camera_path: "/dev/usb_cam1",
  flags: 0,
  width: 800,
  height: 600,
  max_fps: 100,
  camera_matrix: MatrixUtil.buildMatrix([
    [455.332, 0.0, 395.913],
    [0.0, 455.171, 334.9959],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.0529963, -0.0945668, 0.000320554, 0.0000654123, 0.030807495,
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

export default front_left;
