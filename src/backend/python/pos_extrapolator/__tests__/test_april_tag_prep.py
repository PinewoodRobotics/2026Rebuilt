import numpy as np
from numpy.typing import NDArray
import pytest

from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    AprilTagConfig,
    TagDisambiguationMode,
    TagUseImuRotation,
)
from backend.python.common.util.math import from_theta_to_3x3_mat
from backend.generated.proto.python.sensor.apriltags_pb2 import (
    AprilTagData,
    ProcessedTag,
    WorldTags,
)
from backend.generated.thrift.config.common.ttypes import (
    GenericMatrix,
    GenericVector,
    Point3,
)
from backend.python.pos_extrapolator.data_prep import DataPreparer, ExtrapolationContext
from backend.python.pos_extrapolator.position_extrapolator import PositionExtrapolator
from backend.python.pos_extrapolator.preparers import AprilTagPreparer
from backend.python.pos_extrapolator.preparers.AprilTagPreparer import (
    AprilTagPreparerConfig,
    AprilTagDataPreparer,
    AprilTagDataPreparerConfig,
)


def from_theta_to_rotation_state(theta: float) -> NDArray[np.float64]:
    return np.array([0, 0, 0, 0, np.cos(np.radians(theta)), np.sin(np.radians(theta))])


def from_np_to_point3(
    pose: NDArray[np.float64], rotation: NDArray[np.float64]
) -> Point3:
    """
    Convert a pose and rotation from robot coordinates to camera coordinates.
    The pose is a 3D vector in robot coordinates.
    The rotation is a 3x3 matrix in robot coordinates.
    """

    # pose = from_robot_coords_to_camera_coords(pose)
    # rotation = from_robot_rotation_to_camera_rotation(rotation)

    return Point3(
        position=GenericVector(values=[pose[0], pose[1], pose[2]], size=3),
        rotation=GenericMatrix(
            values=[
                [rotation[0, 0], rotation[0, 1], rotation[0, 2]],
                [rotation[1, 0], rotation[1, 1], rotation[1, 2]],
                [rotation[2, 0], rotation[2, 1], rotation[2, 2]],
            ],
            rows=3,
            cols=3,
        ),
    )


def from_robot_coords_to_camera_coords(
    vector: NDArray[np.float64],
) -> NDArray[np.float64]:
    return PositionExtrapolator.CAMERA_OUTPUT_TO_ROBOT_ROTATION.T @ vector


def from_robot_rotation_to_camera_rotation(
    rotation: NDArray[np.float64],
) -> NDArray[np.float64]:
    return (
        PositionExtrapolator.CAMERA_OUTPUT_TO_ROBOT_ROTATION.T
        @ rotation
        @ PositionExtrapolator.CAMERA_OUTPUT_TO_ROBOT_ROTATION
    )


def construct_tag_world(use_imu_rotation: bool = False) -> AprilTagPreparerConfig:
    tags_in_world: dict[int, Point3] = {}
    cameras_in_robot: dict[str, Point3] = {}

    tags_in_world[0] = from_np_to_point3(
        pose=np.array([0, 0, 0]),
        rotation=from_theta_to_3x3_mat(0),
    )

    tags_in_world[1] = from_np_to_point3(
        pose=np.array([1, 0, 0]),
        rotation=from_theta_to_3x3_mat(90),
    )

    tags_in_world[2] = from_np_to_point3(
        pose=np.array([0, 1, 0]),
        rotation=from_theta_to_3x3_mat(180),
    )

    tags_in_world[3] = from_np_to_point3(
        pose=np.array([1, 1, 0]),
        rotation=from_theta_to_3x3_mat(270),
    )

    tags_in_world[4] = from_np_to_point3(
        pose=np.array([0, 1, 0]),
        rotation=from_theta_to_3x3_mat(360),
    )

    cameras_in_robot["camera_1"] = from_np_to_point3(
        pose=np.array([0, 0, 0]),
        rotation=from_theta_to_3x3_mat(0),
    )

    tag_use_imu_rotation = (
        TagUseImuRotation.ALWAYS if use_imu_rotation else TagUseImuRotation.NEVER
    )
    april_tag_config = AprilTagConfig(
        tag_position_config=tags_in_world,
        tag_disambiguation_mode=TagDisambiguationMode.NONE,
        camera_position_config=cameras_in_robot,
        tag_use_imu_rotation=tag_use_imu_rotation,
        disambiguation_time_window_s=0.1,
    )

    return AprilTagPreparerConfig(
        tags_in_world=tags_in_world,
        cameras_in_robot=cameras_in_robot,
        use_imu_rotation=tag_use_imu_rotation,
        april_tag_config=april_tag_config,
    )


def make_april_tag_preparer(
    use_imu_rotation: bool = False,
) -> DataPreparer[AprilTagData, AprilTagDataPreparerConfig]:
    return AprilTagDataPreparer(  # pyright: ignore[reportReturnType]
        AprilTagDataPreparerConfig(construct_tag_world(use_imu_rotation))
    )  # type: ignore


def test_april_tag_prep_one():
    """
    Tests the AprilTagDataPreparer with a single tag.
    The tag is at 0, 0 in the world and the camera is expected to be in the -1, 0 position.
    """

    preparer = make_april_tag_preparer()

    tag_one_R = from_robot_rotation_to_camera_rotation(from_theta_to_3x3_mat(0))
    tag_one_t = from_robot_coords_to_camera_coords(np.array([1, 0, 0]))

    tag_vision_one = AprilTagData(
        world_tags=WorldTags(
            tags=[
                ProcessedTag(
                    id=0,
                    pose_R=tag_one_R.flatten().tolist(),
                    pose_t=tag_one_t.tolist(),
                )
            ]
        )
    )

    output = preparer.prepare_input(tag_vision_one, "camera_1")
    assert output is not None
    inputs = output.get_input_list()
    assert len(inputs) == 1
    assert inputs[0].data.shape == (4,)
    assert np.all(np.isfinite(inputs[0].data))
    # The tag is 1m in front of the camera, tag in world at origin -> robot at (-1, 0).
    assert float(inputs[0].data[0]) == pytest.approx(-1.0, abs=1e-6)
    assert float(inputs[0].data[1]) == pytest.approx(0.0, abs=1e-6)


def test_april_tag_prep_two():
    preparer = make_april_tag_preparer(use_imu_rotation=True)

    tag_one_R = from_robot_rotation_to_camera_rotation(
        from_theta_to_3x3_mat(10)
    )  # noisy
    tag_one_t = from_robot_coords_to_camera_coords(np.array([1, 0, 0]))

    tag_vision_one = AprilTagData(
        world_tags=WorldTags(
            tags=[
                ProcessedTag(
                    id=0,
                    pose_R=tag_one_R.flatten().tolist(),
                    pose_t=tag_one_t.tolist(),
                )
            ]
        )
    )

    output = preparer.prepare_input(
        tag_vision_one,
        "camera_1",
        ExtrapolationContext(
            x=np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]),
            has_gotten_rotation=False,
        ),
    )
    assert output is not None
    assert len(output.get_input_list()) == 1
