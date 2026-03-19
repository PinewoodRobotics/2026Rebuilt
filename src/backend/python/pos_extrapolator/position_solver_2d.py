from dataclasses import dataclass
from typing import TypeVar, final
import numpy as np

from backend.generated.proto.python.sensor.odometry_pb2 import OdometryData
from backend.generated.proto.python.sensor.apriltags_pb2 import AprilTagData
from backend.generated.proto.python.sensor.imu_pb2 import ImuData
from backend.generated.thrift.config.common.ttypes import Point3
from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    AprilTagConfig,
    ImuConfig,
    OdomConfig,
    PosExtrapolator,
    TagUseImuRotation,
)
from backend.generated.thrift.config.camera.ttypes import CameraParameters

TSensorTypes = TypeVar("TSensorTypes", bound=OdometryData | AprilTagData | ImuData)


@dataclass
class TagBasedConfig:
    april_tag_config: AprilTagConfig
    cameras_in_robot: dict[str, Point3]
    use_imu_rotation: TagUseImuRotation
    tags_in_world: dict[int, Point3]


class PositionSolver2d:
    kCameraOutputToRobotRotation = np.array(
        [
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0],
        ]
    )

    def set_tag_based_config(
        self,
        april_tag_config: AprilTagConfig,
        cameras_in_robot: dict[str, Point3],
        use_imu_rotation: TagUseImuRotation,
        tags_in_world: dict[int, Point3],
    ):
        self.tag_based_config = TagBasedConfig(
            april_tag_config=april_tag_config,
            cameras_in_robot=cameras_in_robot,
            use_imu_rotation=use_imu_rotation,
            tags_in_world=tags_in_world,
        )

    def set_odom_config(self, config: OdomConfig):
        self.odom_config = config

    def set_imu_config(self, config: dict[str, ImuConfig]):
        self.imu_config = config

    def __init__(self, config: PosExtrapolator):
        self.general_config = config
        self.odom_config: OdomConfig | None = None
        self.imu_config: dict[str, ImuConfig] | None = None
        self.tag_based_config: TagBasedConfig | None = None
