import {
  AprilDetectionConfig,
  SpecialDetectorType,
} from "generated/thrift/gen-nodejs/apriltag_types";

export const april_tag_detection_config: AprilDetectionConfig = {
  tag_size: 0.17,
  family: "tag36h11",
  nthreads: 4,
  quad_decimate: 1,
  quad_sigma: 0,
  refine_edges: true,
  decode_sharpening: 0.25,
  searchpath: ["apriltags"],
  debug: false,
  post_tag_output_topic: "apriltag/tag",
  send_stats: true,
  stats_topic: "apriltag/stats",
  pi_name_to_special_detector_config: {
    jetson1: {
      type: SpecialDetectorType.GPU_CUDA,
      py_lib_searchpath:
        "/opt/blitz/B.L.I.T.Z/build/release/2.35/aarch64/cpp/cuda-tags-lib/",
    },
  },
};
