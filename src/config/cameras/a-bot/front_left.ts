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
    [455.48175495087486, 0.0, 407.2173573090477],
    [0.0, 455.4341511347836, 335.6999066346699],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.04908778853478117, -0.08285586799580322, -0.0003984055550807527,
    -0.0001698331827697349, 0.022084131827275894,
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
