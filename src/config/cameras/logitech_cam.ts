import {
  CameraType,
  type CameraParameters,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../util/math";

const logitech_cam: CameraParameters = {
  pi_to_run_on: "nathan-hale",
  name: "front_left",
  camera_path: "/dev/usb_cam2",
  flags: 0,
  width: 640,
  height: 480,
  max_fps: 30,
  camera_matrix: MatrixUtil.buildMatrix([
    [643.5526413214271, 0.0, 314.11627942134857],
    [0.0, 643.7371080706604, 235.35269388211123],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    -0.44180630590282977, 0.23535469092748917, -0.0020750769021071484,
    -7.455571357241929e-5, -0.08061071367847858,
  ]),
  exposure_time: 400,
  camera_type: CameraType.ULTRAWIDE_100,
  brightness: 50,
  video_options: {
    send_feed: true,
    overlay_tags: false,
    publication_topic: "camera/logitech/video",
    compression_quality: 90,
    do_compression: true,
  },
};

export default logitech_cam;
