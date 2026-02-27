import numpy as np
import pytest

from backend.generated.proto.python.sensor.apriltags_pb2 import AprilTagData, WorldTags
from backend.generated.proto.python.sensor.imu_pb2 import ImuData
from backend.generated.proto.python.sensor.odometry_pb2 import OdometryData
from backend.generated.thrift.config.common.ttypes import GenericMatrix, GenericVector, Point3
from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    AprilTagConfig,
    ImuConfig,
    OdomConfig,
    OdometryPositionSource,
    TagNoiseAdjustConfig,
    TagUseImuRotation,
)
from backend.python.common.util.math import from_theta_to_3x3_mat
from backend.python.pos_extrapolator.data_prep import DataPreparerManager, ExtrapolationContext
from backend.python.pos_extrapolator.preparers.AprilTagPreparer import (
    AprilTagDataPreparer,
    AprilTagDataPreparerConfig,
    AprilTagPreparerConfig,
)
from backend.python.pos_extrapolator.preparers.ImuDataPreparer import ImuDataPreparerConfig
from backend.python.pos_extrapolator.preparers.OdomDataPreparer import OdomDataPreparerConfig


class _FakeFilter:
    def __init__(self, x: np.ndarray | None = None):
        self.x = x if x is not None else np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def angle_matrix(self) -> np.ndarray:
        c = float(np.cos(self.x[4]))
        s = float(np.sin(self.x[4]))
        return np.array([[c, -s], [s, c]])

    def get_state(self) -> np.ndarray:
        return self.x.copy()

    def angle(self) -> np.ndarray:
        return np.array([np.cos(self.x[4]), np.sin(self.x[4])])


def _ctx(*, x: np.ndarray | None = None, has_gotten_rotation: bool = False) -> ExtrapolationContext:
    return ExtrapolationContext(
        filter=_FakeFilter(x),
        has_gotten_rotation=has_gotten_rotation,
    )


def _point3(p: np.ndarray, R: np.ndarray) -> Point3:
    return Point3(
        position=GenericVector(values=[float(p[0]), float(p[1]), float(p[2])], size=3),
        rotation=GenericMatrix(
            values=[
                [float(R[0, 0]), float(R[0, 1]), float(R[0, 2])],
                [float(R[1, 0]), float(R[1, 1]), float(R[1, 2])],
                [float(R[2, 0]), float(R[2, 1]), float(R[2, 2])],
            ],
            rows=3,
            cols=3,
        ),
    )


def _april_cfg(
    tags_in_world: dict[int, Point3],
    cameras_in_robot: dict[str, Point3],
    use_imu_rotation: TagUseImuRotation,
) -> AprilTagConfig:
    return AprilTagConfig(
        tag_position_config=tags_in_world,
        camera_position_config=cameras_in_robot,
        tag_use_imu_rotation=use_imu_rotation,
        noise_change_modes=[],
        tag_noise_adjust_config=TagNoiseAdjustConfig(
            weight_per_m_from_distance_from_tag=0.0,
            weight_per_degree_from_angle_error_tag=0.0,
            weight_per_confidence_tag=0.0,
        ),
    )


def sample_imu() -> ImuData:
    imu = ImuData()
    imu.position.position.x = 1.0
    imu.position.position.y = 2.0
    imu.velocity.x = 3.0
    imu.velocity.y = 4.0
    imu.position.direction.x = 0.6
    imu.position.direction.y = 0.8
    imu.angularVelocityXYZ.z = 0.5
    return imu


def sample_odom() -> OdometryData:
    odom = OdometryData()
    odom.position.position.x = 10.0
    odom.position.position.y = 20.0
    odom.position_change.x = 1.0
    odom.position_change.y = 2.0
    odom.velocity.x = 5.0
    odom.velocity.y = 6.0
    odom.position.direction.x = 0.0
    odom.position.direction.y = 1.0
    return odom


@pytest.mark.parametrize(
    "use_position,use_velocity,use_rotation,expected_len",
    [
        (False, False, False, 0),
        (True, False, False, 2),
        (False, True, False, 2),
        (False, False, True, 2),
        (True, True, False, 4),
        (True, True, True, 6),
    ],
)
def test_imu_preparer_value_selection_and_shapes(
    use_position: bool, use_velocity: bool, use_rotation: bool, expected_len: int
):
    DataPreparerManager.set_config(
        ImuData,
        ImuDataPreparerConfig(
            {
                "imu0": ImuConfig(
                    use_rotation=use_rotation,
                    use_position=use_position,
                    use_velocity=use_velocity,
                )
            }
        ),
    )

    mgr = DataPreparerManager()
    ctx = _ctx()
    out = mgr.prepare_data(sample_imu(), "imu0", ctx)
    assert out is not None
    assert len(out) == 1
    assert out[0].get_input().shape == (expected_len,)

    state = ctx.filter.get_state()
    H = out[0].jacobian_h(state) if out[0].jacobian_h is not None else None
    hx = out[0].hx(state) if out[0].hx is not None else None
    assert H is not None and hx is not None
    assert H.shape[0] == expected_len
    assert hx.shape == (expected_len,)


def test_imu_preparer_missing_sensor_id_raises_keyerror():
    DataPreparerManager.set_config(
        ImuData,
        ImuDataPreparerConfig(
            {
                "imu0": ImuConfig(
                    use_rotation=True,
                    use_position=False,
                    use_velocity=True,
                )
            }
        ),
    )
    mgr = DataPreparerManager()
    with pytest.raises(KeyError):
        _ = mgr.prepare_data(sample_imu(), "missing", _ctx())


def test_odom_preparer_absolute_includes_position_and_rotates_velocity():
    DataPreparerManager.set_config(
        OdometryData,
        OdomDataPreparerConfig(
            OdomConfig(
                position_source=OdometryPositionSource.ABSOLUTE, use_rotation=False
            )
        ),
    )
    mgr = DataPreparerManager()
    ctx = _ctx(x=np.array([0.0, 0.0, 0.0, 0.0, np.pi / 2, 0.0]))

    out = mgr.prepare_data(sample_odom(), "odom", ctx)
    assert out is not None
    vals = out[0].get_input().tolist()
    assert vals[0] == pytest.approx(10.0)
    assert vals[1] == pytest.approx(20.0)
    assert vals[2] == pytest.approx(-6.0)
    assert vals[3] == pytest.approx(5.0)


def test_odom_preparer_abs_change_updates_position_by_delta():
    DataPreparerManager.set_config(
        OdometryData,
        OdomDataPreparerConfig(
            OdomConfig(
                position_source=OdometryPositionSource.ABS_CHANGE, use_rotation=False
            )
        ),
    )
    mgr = DataPreparerManager()
    ctx = _ctx(x=np.array([100.0, 200.0, 0.0, 0.0, 0.0, 0.0]))
    out = mgr.prepare_data(sample_odom(), "odom", ctx)
    assert out is not None
    vals = out[0].get_input().tolist()
    assert vals[0] == pytest.approx(101.0)
    assert vals[1] == pytest.approx(202.0)


@pytest.mark.xfail(
    reason="OdomDataPreparer.calc_next_absolute_position currently applies raw delta without rotating into world frame"
)
def test_odom_preparer_abs_change_should_rotate_position_delta():
    DataPreparerManager.set_config(
        OdometryData,
        OdomDataPreparerConfig(
            OdomConfig(
                position_source=OdometryPositionSource.ABS_CHANGE, use_rotation=False
            )
        ),
    )
    mgr = DataPreparerManager()
    ctx = _ctx(x=np.array([100.0, 200.0, 0.0, 0.0, np.pi / 2, 0.0]))
    odom = sample_odom()
    odom.position_change.x = 1.0
    odom.position_change.y = 2.0
    out = mgr.prepare_data(odom, "odom", ctx)
    assert out is not None
    vals = out[0].get_input().tolist()
    assert vals[0] == pytest.approx(98.0)
    assert vals[1] == pytest.approx(201.0)


def test_april_tag_preparer_raises_on_raw_tags():
    tags_in_world = {0: _point3(np.array([0.0, 0.0, 0.0]), from_theta_to_3x3_mat(0))}
    cameras_in_robot = {
        "cam0": _point3(np.array([0.0, 0.0, 0.0]), from_theta_to_3x3_mat(0))
    }

    april_tag_cfg = _april_cfg(tags_in_world, cameras_in_robot, TagUseImuRotation.NEVER)
    preparer = AprilTagDataPreparer(
        AprilTagDataPreparerConfig(
            AprilTagPreparerConfig(
                tags_in_world=tags_in_world,
                cameras_in_robot=cameras_in_robot,
                use_imu_rotation=TagUseImuRotation.NEVER,
                april_tag_config=april_tag_cfg,
            )
        )
    )

    data = AprilTagData()
    data.raw_tags.corners.extend([])
    with pytest.raises(ValueError):
        _ = preparer.prepare_input(data, "cam0", _ctx())


def test_april_tag_preparer_skips_unknown_tag_ids_and_returns_empty_input_list():
    tags_in_world = {0: _point3(np.array([0.0, 0.0, 0.0]), from_theta_to_3x3_mat(0))}
    cameras_in_robot = {
        "cam0": _point3(np.array([0.0, 0.0, 0.0]), from_theta_to_3x3_mat(0))
    }
    april_tag_cfg = _april_cfg(tags_in_world, cameras_in_robot, TagUseImuRotation.NEVER)
    preparer = AprilTagDataPreparer(
        AprilTagDataPreparerConfig(
            AprilTagPreparerConfig(
                tags_in_world=tags_in_world,
                cameras_in_robot=cameras_in_robot,
                use_imu_rotation=TagUseImuRotation.NEVER,
                april_tag_config=april_tag_cfg,
            )
        )
    )

    data = AprilTagData(world_tags=WorldTags(tags=[]))
    out = preparer.prepare_input(data, "cam0", _ctx())
    assert out is not None
    assert out == []


@pytest.mark.xfail(
    reason="AprilTagDataPreparer.should_use_imu_rotation appears inverted for WHILE_NO_OTHER_ROTATION_DATA"
)
def test_april_tag_preparer_while_no_other_rotation_should_use_imu_before_rotation_data():
    tags_in_world = {0: _point3(np.array([0.0, 0.0, 0.0]), from_theta_to_3x3_mat(0))}
    cameras_in_robot = {
        "cam0": _point3(np.array([0.0, 0.0, 0.0]), from_theta_to_3x3_mat(0))
    }
    april_tag_cfg = _april_cfg(
        tags_in_world, cameras_in_robot, TagUseImuRotation.WHILE_NO_OTHER_ROTATION_DATA
    )
    preparer = AprilTagDataPreparer(
        AprilTagDataPreparerConfig(
            AprilTagPreparerConfig(
                tags_in_world=tags_in_world,
                cameras_in_robot=cameras_in_robot,
                use_imu_rotation=TagUseImuRotation.WHILE_NO_OTHER_ROTATION_DATA,
                april_tag_config=april_tag_cfg,
            )
        )
    )
    assert preparer.should_use_imu_rotation(_ctx(has_gotten_rotation=False)) is True
