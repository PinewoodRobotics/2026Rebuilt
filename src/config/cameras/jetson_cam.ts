import {
  CameraParameters,
  CameraType,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../util/math";

const jetson_cam: CameraParameters = {
  pi_to_run_on: "jetson1",
  name: "jetson_cam",
  camera_path: "/dev/video0",
  flags: 0,
  width: 640,
  height: 480,
  max_fps: 60,
  camera_matrix: MatrixUtil.buildMatrix([
    [685.088010528533, 0.0, 400.456913426101],
    [0.0, 685.286311116306, 283.611674040712],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.0370949377097917, 0.041319913100527, -0.00128168607814328,
    -0.00128042296949747, -0.298591139932664,
  ]),
  exposure_time: 10,
  camera_type: 0 as CameraType,
  video_options: {
    send_feed: true,
    publication_topic: "camera/jetson/video",
    compression_quality: 90,
    do_compression: false,
    overlay_tags: false,
  },
};

export default jetson_cam;
