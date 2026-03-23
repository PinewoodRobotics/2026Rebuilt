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
    [456.65493913942714, 0.0, 404.57079516684973],
    [0.0, 456.6703454125577, 320.74532359781733],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.051799051290726865, -0.0848067760485244, -0.00019809075335798206,
    0.0001768446674174986, 0.02136733821378993,
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
