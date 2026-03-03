from dataclasses import dataclass
import math
from typing import TYPE_CHECKING
import numpy as np
from numpy.typing import NDArray
from backend.python.common.util.math import (
    create_transformation_matrix,
    from_float_list,
    get_np_from_matrix,
    get_np_from_vector,
    get_robot_in_world,
    get_translation_rotation_components,
    make_transformation_matrix_p_d,
)
from backend.generated.proto.python.sensor.apriltags_pb2 import (
    AprilTagData,
    ProcessedTag,
)
from backend.generated.thrift.config.common.ttypes import Point3
from backend.generated.thrift.config.kalman_filter.ttypes import KalmanFilterSensorType
from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    AprilTagConfig,
    TagNoiseAdjustMode,
    TagUseImuRotation,
)
from backend.python.pos_extrapolator.data_prep import (
    ConfigProvider,
    DataPreparer,
    DataPreparerManager,
    ExtrapolationContext,
    KalmanFilterInput,
)
from backend.python.pos_extrapolator.filters.extended_kalman_filter import T_EKF
from backend.python.pos_extrapolator.position_extrapolator import PositionExtrapolator

# from typing import override

# rotation_angle_rad = np.atan2( <- correct rotation theta angle
#    render_direction_vector[1] /*y*/, render_direction_vector[0] /*x*/
# )


@dataclass
class AprilTagPreparerConfig:
    tags_in_world: dict[int, Point3]
    cameras_in_robot: dict[str, Point3]
    use_imu_rotation: TagUseImuRotation
    april_tag_config: AprilTagConfig


class AprilTagDataPreparerConfig(ConfigProvider[AprilTagPreparerConfig]):
    pass


@DataPreparerManager.register(proto_type=AprilTagData)
class AprilTagDataPreparer(DataPreparer[AprilTagData, AprilTagDataPreparerConfig]):
    def __init__(self, config: AprilTagDataPreparerConfig):
        super().__init__(config)
        self.config: AprilTagDataPreparerConfig = config

        conf = self.config.get_config()

        self.tags_in_world: dict[int, Point3] = conf.tags_in_world
        self.cameras_in_robot: dict[str, Point3] = conf.cameras_in_robot
        self.use_imu_rotation: TagUseImuRotation = conf.use_imu_rotation
        self.april_tag_config: AprilTagConfig = conf.april_tag_config

        self.noise_change_modes = self.april_tag_config.noise_change_modes

    def should_use_imu_rotation(self, context: ExtrapolationContext) -> bool:
        if self.use_imu_rotation == TagUseImuRotation.NEVER:
            return False

        if self.use_imu_rotation == TagUseImuRotation.ALWAYS:
            return True

        if self.use_imu_rotation == TagUseImuRotation.WHILE_NO_OTHER_ROTATION_DATA:
            return context.has_gotten_rotation

        return False

    def get_noise_change(
        self, x_hat: NDArray[np.float64], tag_detection: ProcessedTag
    ) -> tuple[float, float]:
        total_add = 0.0
        total_mult = 1.0

        if TagNoiseAdjustMode.ADD_WEIGHT_PER_M_DISTANCE_TAG in self.noise_change_modes:
            distance = np.linalg.norm(x_hat[0:2])
            weight = (
                self.april_tag_config.tag_noise_adjust_config.weight_per_m_from_distance_from_tag
                * distance
            )

            total_add += float(weight)
        if TagNoiseAdjustMode.ADD_WEIGHT_PER_TAG_CONFIDENCE in self.noise_change_modes:
            total_add += float(
                tag_detection.confidence
                * self.april_tag_config.tag_noise_adjust_config.weight_per_confidence_tag
            )

        return float(total_add), float(total_mult)

    # @override
    def get_data_type(self) -> type[AprilTagData]:
        return AprilTagData

    # @override
    def _used_indices(self) -> list[bool]:
        used_indices: list[bool] = []

        used_indices.extend([True] * 2)
        used_indices.extend([False] * 2)
        used_indices.extend([True])
        used_indices.extend([False])

        return used_indices

    # @override
    def _prepare(
        self,
        data: AprilTagData,
        sensor_id: str,
        context: ExtrapolationContext | None = None,
    ) -> list[KalmanFilterInput] | KalmanFilterInput | None:
        assert context is not None
        if data.WhichOneof("data") == "raw_tags":
            raise ValueError(
                "Tried to insert AprilTagData with raw tags, but tags are not in processed format"
            )

        input_list: list[KalmanFilterInput] = []
        for tag in data.world_tags.tags:
            tag_id = tag.id
            if tag_id not in self.tags_in_world:
                continue

            T_camera_in_robot = create_transformation_matrix(
                rotation_matrix=get_np_from_matrix(
                    self.cameras_in_robot[sensor_id].rotation
                ),
                translation_vector=get_np_from_vector(
                    self.cameras_in_robot[sensor_id].position
                ),
            )
            T_tag_in_world = create_transformation_matrix(
                rotation_matrix=get_np_from_matrix(self.tags_in_world[tag_id].rotation),
                translation_vector=get_np_from_vector(
                    self.tags_in_world[tag_id].position
                ),
            )

            tag_in_camera_rotation = (
                PositionExtrapolator.CAMERA_OUTPUT_TO_ROBOT_ROTATION
                @ from_float_list(list(tag.pose_R), 3, 3)
                @ PositionExtrapolator.CAMERA_OUTPUT_TO_ROBOT_ROTATION.T
            )

            tag_in_camera_pose = (
                PositionExtrapolator.CAMERA_OUTPUT_TO_ROBOT_ROTATION
                @ np.array(tag.pose_t)
            )

            T_tag_in_camera = create_transformation_matrix(
                rotation_matrix=tag_in_camera_rotation,
                translation_vector=tag_in_camera_pose,
            )

            R_robot_rotation_world: NDArray[np.float64] | None = None
            if self.should_use_imu_rotation(context):
                direction_2d = context.filter.angle()
                direction_3d = np.array([direction_2d[0], direction_2d[1], 0.0])
                R_robot_rotation_world = make_transformation_matrix_p_d(
                    direction_vector=direction_3d,
                )[:3, :3]

            pose, rotation = get_translation_rotation_components(
                get_robot_in_world(
                    T_tag_in_camera=T_tag_in_camera,
                    T_camera_in_robot=T_camera_in_robot,
                    T_tag_in_world=T_tag_in_world,
                    R_robot_rotation_world=R_robot_rotation_world,
                )
            )

            if self.april_tag_config.insert_predicted_global_rotation:
                _, rotation_pred = get_translation_rotation_components(
                    get_robot_in_world(
                        T_tag_in_camera=T_tag_in_camera,
                        T_camera_in_robot=T_camera_in_robot,
                        T_tag_in_world=T_tag_in_world,
                    )
                )

                rotation = rotation_pred

            direction_vector = rotation[0:3, 0]  # extract cos and sin
            angle_rad = np.atan2(direction_vector[1], direction_vector[0])
            datapoint = np.array(
                [
                    pose[0],
                    pose[1],
                    angle_rad,
                ]
            )

            add, mult = self.get_noise_change(datapoint, tag)

            input_list.append(
                KalmanFilterInput(
                    input=datapoint,
                    sensor_id=sensor_id,
                    sensor_type=KalmanFilterSensorType.APRIL_TAG,
                    jacobian_h=T_EKF.generic_jacobian_h(self.get_used_indices()),
                    hx=T_EKF.generic_hx(self.get_used_indices()),
                    R_add=add,
                    R_mult=mult,
                )
            )

        return input_list
