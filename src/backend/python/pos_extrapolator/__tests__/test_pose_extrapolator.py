from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
import pytest

from backend.generated.proto.python.util.position_pb2 import RobotPosition
from backend.generated.thrift.config.kalman_filter.ttypes import KalmanFilterSensorType
from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    AprilTagConfig,
    ImuConfig,
    OdomConfig,
    OdometryPositionSource,
    PosExtrapolator,
    PosExtrapolatorMessageConfig,
    TagDisambiguationMode,
    TagUseImuRotation,
)
from backend.python.pos_extrapolator.data_prep import (
    ExtrapolationContext,
    KalmanFilterInput,
    ProcessedData,
)
from backend.python.pos_extrapolator.position_extrapolator import PositionExtrapolator


@dataclass
class FakeFilterStrategy:
    x: NDArray[np.float64]
    P: NDArray[np.float64]
    confidence: float = 0.5

    insert_calls: int = 0
    last_inserted: KalmanFilterInput | None = None
    last_future_s: float | None = None

    def insert_data(self, data: KalmanFilterInput) -> None:
        self.insert_calls += 1
        self.last_inserted = data

    def get_state(self, future_s: float | None = None) -> NDArray[np.float64]:
        self.last_future_s = future_s
        return self.x

    def get_P(self) -> NDArray[np.float64]:
        return self.P

    def get_confidence(self) -> float:
        return self.confidence


@dataclass
class FakeDataPreparerManager:
    next_return: KalmanFilterInput | None = None
    last_data: object | None = None
    last_sensor_id: str | None = None
    last_context: ExtrapolationContext | None = None
    calls: int = 0

    def prepare_data(
        self, data: object, sensor_id: str, context: ExtrapolationContext | None = None
    ) -> KalmanFilterInput | None:
        self.calls += 1
        self.last_data = data
        self.last_sensor_id = sensor_id
        self.last_context = context
        return self.next_return


def make_min_config(
    *,
    tag_use_imu_rotation: TagUseImuRotation,
    odom_use_rotation: bool,
    imu_use_rotation: bool,
    future_position_prediction_margin_s: float = 0.0,
    imu_sensor_id: str = "imu0",
) -> PosExtrapolator:
    message_config = PosExtrapolatorMessageConfig(
        post_tag_input_topic="test/post_tag_input",
        post_odometry_input_topic="test/post_odom_input",
        post_imu_input_topic="test/post_imu_input",
        post_robot_position_output_topic="test/post_robot_position",
    )

    april_tag_config = AprilTagConfig(
        tag_position_config={},
        tag_disambiguation_mode=TagDisambiguationMode.NONE,
        camera_position_config={},
        tag_use_imu_rotation=tag_use_imu_rotation,
        disambiguation_time_window_s=0.1,
    )

    odom_config = OdomConfig(
        position_source=OdometryPositionSource.DONT_USE,
        use_rotation=odom_use_rotation,
    )

    imu_config = {
        imu_sensor_id: ImuConfig(
            use_rotation=imu_use_rotation,
            use_position=False,
            use_velocity=True,
        )
    }

    # Note: we intentionally leave kalman_filter_config unset (None) because these unit
    # tests use a fake filter strategy and never call validate().
    return PosExtrapolator(
        message_config=message_config,
        enable_imu=True,
        enable_odom=True,
        enable_tags=True,
        april_tag_config=april_tag_config,
        odom_config=odom_config,
        imu_config=imu_config,
        kalman_filter_config=None,  # pyright: ignore[reportArgumentType]
        future_position_prediction_margin_s=future_position_prediction_margin_s,
    )


def make_kfi(
    sensor_type: KalmanFilterSensorType, sensor_id: str = "s0"
) -> KalmanFilterInput:
    return KalmanFilterInput(
        input=ProcessedData(data=np.array([0.0])),
        sensor_id=sensor_id,
        sensor_type=sensor_type,
    )


def make_subject(
    *,
    tag_use_imu_rotation: TagUseImuRotation = TagUseImuRotation.ALWAYS,
    odom_use_rotation: bool = False,
    imu_use_rotation: bool = False,
    future_position_prediction_margin_s: float = 0.0,
    imu_sensor_id: str = "imu0",
    x: NDArray[np.float64] | None = None,
    p_matrix: NDArray[np.float64] | None = None,
    confidence: float = 0.5,
) -> tuple[PositionExtrapolator, FakeFilterStrategy, FakeDataPreparerManager]:
    config = make_min_config(
        tag_use_imu_rotation=tag_use_imu_rotation,
        odom_use_rotation=odom_use_rotation,
        imu_use_rotation=imu_use_rotation,
        future_position_prediction_margin_s=future_position_prediction_margin_s,
        imu_sensor_id=imu_sensor_id,
    )

    if x is None:
        x = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
    if p_matrix is None:
        p_matrix = np.eye(7)

    fake_filter = FakeFilterStrategy(x=x, P=p_matrix, confidence=confidence)
    fake_manager = FakeDataPreparerManager()

    return (
        PositionExtrapolator(
            config=config,
            filter_strategy=fake_filter,  # pyright: ignore[reportArgumentType]
            data_preparer_manager=fake_manager,  # pyright: ignore[reportArgumentType]
        ),
        fake_filter,
        fake_manager,
    )


def test_initial_has_gotten_rotation_true_when_tags_never_use_imu_rotation():
    pe, _, _ = make_subject(tag_use_imu_rotation=TagUseImuRotation.NEVER)
    assert pe.has_gotten_rotation is True


def test_initial_has_gotten_rotation_false_when_tags_use_imu_rotation():
    pe, _, _ = make_subject(tag_use_imu_rotation=TagUseImuRotation.ALWAYS)
    assert pe.has_gotten_rotation is False


def test_insert_sensor_data_passes_context_state_and_flag():
    x = np.array([1.0, 2.0, 3.0, 4.0, 0.5, 0.5, 0.0])
    pe, fake_filter, mgr = make_subject(
        tag_use_imu_rotation=TagUseImuRotation.ALWAYS, x=x
    )
    assert pe.has_gotten_rotation is False

    mgr.next_return = None
    pe.insert_sensor_data(data=object(), sensor_id="any")

    assert mgr.calls == 1
    assert mgr.last_context is not None
    assert np.allclose(mgr.last_context.x, fake_filter.x)
    assert mgr.last_context.has_gotten_rotation is False


def test_insert_sensor_data_noop_when_prepare_data_returns_none():
    pe, fake_filter, mgr = make_subject()
    mgr.next_return = None
    pe.insert_sensor_data(data=object(), sensor_id="any")
    assert fake_filter.insert_calls == 0


def test_april_tag_does_not_set_has_gotten_rotation():
    pe, _, mgr = make_subject(tag_use_imu_rotation=TagUseImuRotation.ALWAYS)
    assert pe.has_gotten_rotation is False

    mgr.next_return = make_kfi(KalmanFilterSensorType.APRIL_TAG, sensor_id="cam0")
    pe.insert_sensor_data(data=object(), sensor_id="cam0")
    assert pe.has_gotten_rotation is False


def test_odom_sets_has_gotten_rotation_only_when_config_use_rotation_true():
    pe1, _, mgr1 = make_subject(
        tag_use_imu_rotation=TagUseImuRotation.ALWAYS,
        odom_use_rotation=False,
    )
    mgr1.next_return = make_kfi(KalmanFilterSensorType.ODOMETRY, sensor_id="odom")
    pe1.insert_sensor_data(data=object(), sensor_id="odom")
    assert pe1.has_gotten_rotation is False

    pe2, _, mgr2 = make_subject(
        tag_use_imu_rotation=TagUseImuRotation.ALWAYS,
        odom_use_rotation=True,
    )
    mgr2.next_return = make_kfi(KalmanFilterSensorType.ODOMETRY, sensor_id="odom")
    pe2.insert_sensor_data(data=object(), sensor_id="odom")
    assert pe2.has_gotten_rotation is True


def test_imu_sets_has_gotten_rotation_only_when_config_use_rotation_true():
    pe1, _, mgr1 = make_subject(
        tag_use_imu_rotation=TagUseImuRotation.ALWAYS,
        imu_use_rotation=False,
        imu_sensor_id="imu0",
    )
    mgr1.next_return = make_kfi(KalmanFilterSensorType.IMU, sensor_id="imu0")
    pe1.insert_sensor_data(data=object(), sensor_id="imu0")
    assert pe1.has_gotten_rotation is False

    pe2, _, mgr2 = make_subject(
        tag_use_imu_rotation=TagUseImuRotation.ALWAYS,
        imu_use_rotation=True,
        imu_sensor_id="imu0",
    )
    mgr2.next_return = make_kfi(KalmanFilterSensorType.IMU, sensor_id="imu0")
    pe2.insert_sensor_data(data=object(), sensor_id="imu0")
    assert pe2.has_gotten_rotation is True


def test_unknown_sensor_type_defaults_to_rotation_gotten_true():
    pe, _, mgr = make_subject(tag_use_imu_rotation=TagUseImuRotation.ALWAYS)
    assert pe.has_gotten_rotation is False

    mgr.next_return = KalmanFilterInput(
        input=ProcessedData(data=np.array([0.0])),
        sensor_id="s",
        sensor_type=999,  # pyright: ignore[reportArgumentType]
    )
    pe.insert_sensor_data(data=object(), sensor_id="s")
    assert pe.has_gotten_rotation is True


def test_get_robot_position_estimate_returns_flattened_list():
    x = np.array([1.0, 2.0, 3.0, 4.0, 0.25, 0.75, 0.5])
    pe, _, _ = make_subject(x=x)
    assert pe.get_robot_position_estimate() == [1.0, 2.0, 3.0, 4.0, 0.25, 0.75, 0.5]


def test_get_robot_position_estimate_passes_future_s_through():
    pe, fake_filter, _ = make_subject()
    _ = pe.get_robot_position_estimate(future_s=1.5)
    assert fake_filter.last_future_s == 1.5


def test_get_robot_position_maps_state_indices_to_proto_fields():
    x = np.array([10.0, 20.0, 1.1, -2.2, 0.6, 0.8, 0.05])
    P = np.eye(7) * 2.0
    pe, fake_filter, _ = make_subject(
        x=x,
        p_matrix=P,
        confidence=0.123,
        future_position_prediction_margin_s=0.25,
    )

    proto: RobotPosition = pe.get_robot_position()

    assert fake_filter.last_future_s == 0.25
    assert float(proto.position_2d.position.x) == pytest.approx(10.0)
    assert float(proto.position_2d.position.y) == pytest.approx(20.0)
    assert float(proto.position_2d.velocity.x) == pytest.approx(1.1)
    assert float(proto.position_2d.velocity.y) == pytest.approx(-2.2)
    assert float(proto.position_2d.direction.x) == pytest.approx(0.6)
    assert float(proto.position_2d.direction.y) == pytest.approx(0.8)
    assert float(proto.position_2d.rotation_speed_rad_s) == pytest.approx(0.05)
    assert float(proto.confidence) == pytest.approx(0.123)


def test_get_position_covariance_flattens_matrix():
    P = np.arange(49, dtype=float).reshape(7, 7)
    pe, _, _ = make_subject(p_matrix=P)
    flat = pe.get_position_covariance()
    assert len(flat) == 49
    assert flat[0] == 0.0
    assert flat[-1] == 48.0


def test_get_confidence_is_forwarded():
    pe, _, _ = make_subject(confidence=0.9)
    assert pe.get_confidence() == 0.9
