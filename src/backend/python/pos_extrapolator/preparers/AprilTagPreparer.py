from dataclasses import dataclass
import math
import numpy as np
from numpy.typing import NDArray
from backend.python.common.debug.logger import debug
from backend.python.common.util.math import (
    create_transformation_matrix,
    from_float_list,
    get_np_from_matrix,
    get_np_from_vector,
    get_robot_in_world,
    get_translation_rotation_components,
    make_3d_rotation_from_yaw,
    make_transformation_matrix_p_d,
    transform_matrix_to_size,
    transform_vector_to_size,
)
from backend.generated.proto.python.sensor.apriltags_pb2 import AprilTagData
from backend.generated.proto.python.sensor.imu_pb2 import ImuData
from backend.generated.proto.python.sensor.odometry_pb2 import OdometryData
from backend.generated.thrift.config.common.ttypes import Point3
from backend.generated.thrift.config.kalman_filter.ttypes import KalmanFilterSensorType
from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    AprilTagConfig,
    ImuConfig,
    OdomConfig,
    TagNoiseAdjustConfig,
    TagNoiseAdjustMode,
    TagUseImuRotation,
)
from backend.python.pos_extrapolator.data_prep import (
    ConfigProvider,
    DataPreparer,
    DataPreparerManager,
    ExtrapolationContext,
    KalmanFilterInput,
    ProcessedData,
)
from backend.python.pos_extrapolator.filters.gate.mahalanobis import (
    mahalanobis_distance,
)
from backend.python.pos_extrapolator.position_extrapolator import PositionExtrapolator


def _angle_difference_deg(x_hat: NDArray[np.float64], x: NDArray[np.float64]) -> float:
    measured_angle = math.atan2(float(x_hat[3]), float(x_hat[2]))
    state_angle = math.atan2(float(x[5]), float(x[4]))
    diff = measured_angle - state_angle
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return abs(math.degrees(diff))


def _distance_difference_m(x_hat: NDArray[np.float64], x: NDArray[np.float64]) -> float:
    return np.sqrt((x_hat[0] - x[0]) ** 2 + (x_hat[1] - x[1]) ** 2)


@dataclass
class AprilTagPreparerConfig:
    tags_in_world: dict[int, Point3]
    cameras_in_robot: dict[str, Point3]
    use_imu_rotation: TagUseImuRotation
    april_tag_config: AprilTagConfig


class AprilTagDataPreparerConfig(ConfigProvider[AprilTagPreparerConfig]):
    def __init__(self, config: AprilTagPreparerConfig):
        self.config = config

    def get_config(self) -> AprilTagPreparerConfig:
        return self.config


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

        self.tag_noise_adjust_mode = self.april_tag_config.tag_noise_adjust_mode

        self.tag_noise_adjust_config: TagNoiseAdjustConfig = (
            self.april_tag_config.tag_noise_adjust_config
        )

    def get_data_type(self) -> type[AprilTagData]:
        return AprilTagData

    def get_used_indices(self) -> list[bool]:
        used_indices: list[bool] = []

        used_indices.extend([True] * 2)
        used_indices.extend([False] * 2)
        used_indices.extend([True] * 2)
        used_indices.extend([False])

        return used_indices

    def jacobian_h(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        return transform_matrix_to_size(self.get_used_indices(), np.eye(7))

    def hx(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        return transform_vector_to_size(x, self.get_used_indices())

    def should_use_imu_rotation(self, context: ExtrapolationContext | None) -> bool:
        if context is None:
            return False

        if self.use_imu_rotation == TagUseImuRotation.ALWAYS:
            return True

        if self.use_imu_rotation == TagUseImuRotation.UNTIL_FIRST_NON_TAG_ROTATION:
            return context.has_gotten_rotation

        return False

    def get_weight_add_config(
        self,
        *,
        x_hat: NDArray[np.float64],
        x: NDArray[np.float64] | None,
        distance_from_tag_m: float,
        tag_confidence: float,
    ) -> tuple[float, float]:
        add = 0.0
        multiplier = 1.0

        for mode in self.tag_noise_adjust_mode:
            if mode == TagNoiseAdjustMode.ADD_WEIGHT_PER_M_DISTANCE_TAG:
                weight = (
                    self.tag_noise_adjust_config.weight_per_m_from_distance_from_tag
                )
                if weight is not None:
                    add += distance_from_tag_m * weight

            elif mode == TagNoiseAdjustMode.ADD_WEIGHT_PER_DEGREE_ERROR_ANGLE_TAG:
                weight = (
                    self.tag_noise_adjust_config.weight_per_degree_from_angle_error_tag
                )
                if weight is not None and x is not None:
                    add += _angle_difference_deg(x_hat, x) * weight

            elif mode == TagNoiseAdjustMode.ADD_WEIGHT_PER_TAG_CONFIDENCE:
                weight = self.tag_noise_adjust_config.weight_per_confidence_tag
                if (
                    weight is not None
                    and tag_confidence is not None
                    and np.isfinite(tag_confidence)
                ):
                    confidence_term = max(0.0, float(tag_confidence))
                    add += confidence_term * weight
            elif mode == TagNoiseAdjustMode.MULTIPLY_POW_BY_M_DISTANCE_FROM_TAG:
                multiplier += (
                    distance_from_tag_m
                    ** self.tag_noise_adjust_config.pow_distance_from_tag_coef
                    * self.tag_noise_adjust_config.multiply_coef_m_distance_from_tag
                )

        return multiplier, add

    def prepare_input(
        self,
        data: AprilTagData,
        sensor_id: str,
        context: ExtrapolationContext | None = None,
    ) -> KalmanFilterInput | None:
        if data.WhichOneof("data") == "raw_tags":
            raise ValueError("Tags are not in processed format")

        input_list: list[ProcessedData] = []
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
                assert context is not None
                R_robot_rotation_world = make_transformation_matrix_p_d(
                    position=np.array([0, 0, 0]),
                    direction_vector=np.array([context.x[4], context.x[5], 0]),
                )[:3, :3]

            render_pose, render_rotation = get_translation_rotation_components(
                get_robot_in_world(
                    T_tag_in_camera=T_tag_in_camera,
                    T_camera_in_robot=T_camera_in_robot,
                    T_tag_in_world=T_tag_in_world,
                    R_robot_rotation_world=R_robot_rotation_world,
                )
            )

            render_direction_vector = render_rotation[0:3, 0]
            # rotation_angle_rad = np.atan2( <- correct rotation theta angle
            #    render_direction_vector[1] /*y*/, render_direction_vector[0] /*x*/
            # )

            datapoint = np.array(
                [
                    render_pose[0],
                    render_pose[1],
                    render_direction_vector[0],
                    render_direction_vector[1],
                ]
            )

            multiplier, add = self.get_weight_add_config(
                x_hat=datapoint,
                x=context.x if context is not None else None,
                distance_from_tag_m=float(np.linalg.norm(tag_in_camera_pose)),
                tag_confidence=tag.confidence,
            )

            input_list.append(
                ProcessedData(data=datapoint, R_multipl=multiplier, R_add=add)
            )

        return KalmanFilterInput(
            input=input_list,
            sensor_id=sensor_id,
            sensor_type=KalmanFilterSensorType.APRIL_TAG,
            jacobian_h=self.jacobian_h,
            hx=self.hx,
        )
