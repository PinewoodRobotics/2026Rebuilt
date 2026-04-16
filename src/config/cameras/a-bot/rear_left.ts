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
    [457.1759, 0.0, 403.1777],
    [0.0, 457.1069, 340.0624],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.0533, -0.09247, -0.0002978, -0.000189, 0.0283,
  ]),
  exposure_time: 30,
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
