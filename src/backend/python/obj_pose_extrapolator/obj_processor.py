from dataclasses import dataclass
from enum import Enum
import time
from backend.generated.thrift.config.obj_pose_extrapolator.ttypes import (
    ObjPoseExtrapolatorConfig,
)
from backend.python.obj_pose_extrapolator.kalman_filter import (
    ObjPersonalKalmanFilter,
)
from backend.generated.proto.python.sensor.inference_pb2 import InferenceData
import numpy as np
from backend.generated.proto.python.util.position_pb2 import RobotPosition
from backend.generated.thrift.config.camera.ttypes import CameraParameters
from backend.python.common.util.math import get_np_from_matrix, get_np_from_vector
import cv2


@dataclass
class TimedObjectPosition:
    last_update_time: float
    position: ObjPersonalKalmanFilter


class ExtrapolationStrategy(Enum):
    AVG_SIZE = "avg_size"
    # PROVIDED_DEPTH = "provided_depth"
    # LIDAR = "lidar"


class ObjPositionProcessor:
    def __init__(
        self, config: ObjPoseExtrapolatorConfig, cameras: dict[str, CameraParameters]
    ):
        self.config = config
        self.object_dimensions = config.object_dimensions
        self.robots_on_field: list[TimedObjectPosition] = []
        self.max_time_disappear_seconds = config.time_disappear_seconds
        self.cameras = cameras
        # A 2D transformation matrix for pose (x, y, theta)
        # Format: [[cosθ, -sinθ, tx], [sinθ, cosθ, ty], [0, 0, 1]]
        self.current_robot_pose_matrix = np.eye(3)

    def tick(self) -> list[list[float]]:
        self.robots_on_field = [
            robot
            for robot in self.robots_on_field
            if time.time() - robot.last_update_time < self.max_time_disappear_seconds
        ]
        return [robot.position.get_state() for robot in self.robots_on_field]

    def insert_robot_position(self, position: RobotPosition):
        self.current_robot_pose_matrix = np.array(
            [
                [
                    np.cos(position.position_2d.direction.x),
                    -np.sin(position.position_2d.direction.x),
                    position.position_2d.position.x,
                ],
                [
                    np.sin(position.position_2d.direction.x),
                    np.cos(position.position_2d.direction.x),
                    position.position_2d.position.y,
                ],
                [0, 0, 1],
            ]
        )

    def insert_data(
        self,
        data: InferenceData,
        camera_id: str,
        extrapolation_strategy: ExtrapolationStrategy = ExtrapolationStrategy.AVG_SIZE,
        depth_estimate: (
            float | None
        ) = None,  # todo: add this to help with depth estimation
    ):
        if data.WhichOneof("data") != "raw_inferences":
            return

        camera_matrix, dist_coeff = self._get_camera_intrinsics(camera_id)

        for inference in data.raw_inferences.inferences_2d:
            if inference.label not in self.object_dimensions:
                continue

            dimensions = self.object_dimensions[inference.label]

            image_points = self._image_points_from_inference(inference)
            if image_points is None:
                continue

            object_points = self._object_points_from_dimensions(dimensions)
            t_cam = self._solve_tvec_in_camera(
                object_points=object_points,
                image_points=image_points,
                camera_matrix=camera_matrix,
                dist_coeff=dist_coeff,
            )
            if t_cam is None:
                continue

            robot_xy = self._camera_t_to_robot_xy(t_cam)
            obj_world_xy = self._robot_xy_to_world_xy(robot_xy)

            now = time.time()

            best_i, best_dist = self._find_nearest_track_index(obj_world_xy)
            if best_i is not None and best_dist <= float(
                self.config.distance_same_object_threshold_meters
            ):
                self._update_track(self.robots_on_field[best_i], obj_world_xy, now)
            else:
                self._create_track(obj_world_xy, now)

    def _get_camera_intrinsics(self, camera_id: str) -> tuple[np.ndarray, np.ndarray]:
        camera_matrix = get_np_from_matrix(self.cameras[camera_id].camera_matrix)
        dist_coeff = get_np_from_vector(self.cameras[camera_id].dist_coeff)
        return camera_matrix, dist_coeff

    def _image_points_from_inference(self, inference) -> np.ndarray | None:
        # `corners` should be 4 points: [tl, tr, br, bl]
        if len(inference.corners) != 4:
            return None
        return np.array([[p.x, p.y] for p in inference.corners], dtype=np.float32)

    def _object_points_from_dimensions(self, dimensions) -> np.ndarray:
        half_w = float(dimensions.width_meters) / 2.0
        half_h = float(dimensions.height_meters) / 2.0
        return np.array(
            [
                [-half_w, -half_h, 0.0],  # top-left
                [half_w, -half_h, 0.0],  # top-right
                [half_w, half_h, 0.0],  # bottom-right
                [-half_w, half_h, 0.0],  # bottom-left
            ],
            dtype=np.float32,
        )

    def _solve_tvec_in_camera(
        self,
        *,
        object_points: np.ndarray,
        image_points: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeff: np.ndarray,
    ) -> np.ndarray | None:
        ok, _rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeff,
            flags=cv2.SOLVEPNP_IPPE,
            useExtrinsicGuess=False,
        )
        if not ok:
            return None
        return np.asarray(tvec, dtype=np.float64).reshape(3)

    def _camera_t_to_robot_xy(self, t_cam: np.ndarray) -> np.ndarray:
        # OpenCV camera coords: x right, y down, z forward
        # Robot 2D frame: x forward, y left
        return np.array([t_cam[2], -t_cam[0]], dtype=np.float64)

    def _robot_xy_to_world_xy(self, robot_xy: np.ndarray) -> np.ndarray:
        world_h = self.current_robot_pose_matrix @ np.array(
            [robot_xy[0], robot_xy[1], 1.0], dtype=np.float64
        )
        return world_h[:2]

    def _find_nearest_track_index(
        self, world_xy: np.ndarray
    ) -> tuple[int | None, float]:
        best_i: int | None = None
        best_dist = float("inf")
        for i, track in enumerate(self.robots_on_field):
            state = track.position.get_state()
            dx = world_xy[0] - state[0]
            dy = world_xy[1] - state[1]
            d = float(np.hypot(dx, dy))
            if d < best_dist:
                best_dist = d
                best_i = i
        return best_i, best_dist

    def _update_track(
        self, track: TimedObjectPosition, world_xy: np.ndarray, now: float
    ):
        dt = max(1e-3, now - track.last_update_time)
        prev = track.position.get_state()
        vx = float((world_xy[0] - prev[0]) / dt)
        vy = float((world_xy[1] - prev[1]) / dt)
        track.position.insert_sensor_data(
            [float(world_xy[0]), float(world_xy[1]), vx, vy]
        )
        track.last_update_time = now

    def _create_track(self, world_xy: np.ndarray, now: float):
        kf = ObjPersonalKalmanFilter(
            self.config.kalman_filter_config,
            [float(world_xy[0]), float(world_xy[1]), 0.0, 0.0],
        )
        self.robots_on_field.append(TimedObjectPosition(now, kf))
