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
    [457.0125746528886, 0.0, 403.48780857304496],
    [0.0, 457.1140457905773, 341.0084282297518],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.050325253188570965, -0.08166111387015701, 0.0003566193011872036,
    5.781569739546963e-6, 0.020389826651388315,
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
