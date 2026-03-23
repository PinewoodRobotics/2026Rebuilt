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
    [456.5485804798424, 0.0, 396.1023879358309],
    [0.0, 456.5795311820434, 335.39613293750654],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.05242766381099558, -0.08722732273491818, 0.00012693974436409078,
    0.00026288865480920254, 0.024429768437823022,
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
