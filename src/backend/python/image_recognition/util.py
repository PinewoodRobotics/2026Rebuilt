from typing import cast
import numpy as np
from numpy.typing import NDArray

import cv2
from ultralytics.engine.results import Results

from backend.generated.proto.python.Inference_pb2 import (
    ProcessedInference,
    RawInference,
)
from backend.generated.proto.python.util.position_pb2 import Rotation3d
from backend.generated.proto.python.util.vector_pb2 import Vector2, Vector3


def preprocess_frame(frame: NDArray[np.uint8]) -> NDArray[np.uint8]:
    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).astype(np.uint8)


def to_raw_inference(result: Results) -> list[RawInference]:
    """
    Convert YOLO Results object to a list of RawInference objects.
    Handles cases where there are zero or multiple detections.
    """
    if result.boxes is None or len(result.boxes) == 0:
        return []

    raw_inferences: list[RawInference] = []
    for i in range(len(result.boxes)):
        box = result.boxes[i]
        raw_inferences.append(
            RawInference(
                confidence=box.conf.item(),
                class_name=result.names[int(box.cls.item())],
                class_id=int(box.cls.item()),
                bounding_box=[
                    Vector2(x=float(box.xyxy[0][0]), y=float(box.xyxy[0][1])),
                    Vector2(x=float(box.xyxy[0][2]), y=float(box.xyxy[0][3])),
                ],
            )
        )
    return raw_inferences


def get_box_corners(result: RawInference) -> list[NDArray[np.float32]]:
    # bounding_box stores [top-left, bottom-right]: [x1, y1], [x2, y2]
    # Return [top-left, top-right, bottom-right, bottom-left] for consistency
    x1, y1 = result.bounding_box[0].x, result.bounding_box[0].y
    x2, y2 = result.bounding_box[1].x, result.bounding_box[1].y
    return [
        np.array([x1, y1], dtype=np.float32),  # top-left
        np.array([x2, y1], dtype=np.float32),  # top-right
        np.array([x2, y2], dtype=np.float32),  # bottom-right
        np.array([x1, y2], dtype=np.float32),  # bottom-left
    ]


def post_process_results(
    results: list[RawInference],
    camera_matrix: NDArray[np.float64],
    dist_coeff: NDArray[np.float64],
    size_x: float,
    size_y: float,
) -> list[ProcessedInference]:
    """
    Post-process raw inference results to compute 3D pose using solvePnP.
    Skips detections where solvePnP fails.
    """
    processed_inferences: list[ProcessedInference] = []
    for result in results:
        try:
            R, tvec = solve_pnp(
                get_box_corners(result), camera_matrix, dist_coeff, size_x, size_y
            )
            processed_inferences.append(
                ProcessedInference(
                    position=Vector3(
                        x=float(tvec[0]),
                        y=float(tvec[1]),
                        z=float(tvec[2]),
                    ),
                    rotation=Rotation3d(
                        directionX=Vector3(
                            x=float(R[0, 0]),
                            y=float(R[1, 0]),
                            z=float(R[2, 0]),
                        ),
                        directionY=Vector3(
                            x=float(R[0, 1]),
                            y=float(R[1, 1]),
                            z=float(R[2, 1]),
                        ),
                        directionZ=Vector3(
                            x=float(R[0, 2]),
                            y=float(R[1, 2]),
                            z=float(R[2, 2]),
                        ),
                    ),
                )
            )
        except (RuntimeError, ValueError):
            # Skip detections where solvePnP fails (e.g., invalid corners, degenerate cases)
            continue

    return processed_inferences


def solve_pnp(
    box_corners: NDArray[np.float32] | list[NDArray[np.float32]],
    camera_matrix: NDArray[np.float64],
    dist_coeff: NDArray[np.float64],
    size_x: float,
    size_y: float,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    Solve PnP problem for a rectangular object.

    Args:
        box_corners: List of 4 corner points as (x, y) coordinates in image space.
                    Order: [top-left, top-right, bottom-right, bottom-left]
        camera_matrix: Camera intrinsic matrix
        dist_coeff: Distortion coefficients
        size_x: Object width in real-world units
        size_y: Object height in real-world units

    Returns:
        Tuple of (rotation_matrix, translation_vector)
    """
    if isinstance(box_corners, list):
        # Convert list of arrays to a single (4, 2) array
        box_corners = np.array(
            [corner.flatten() for corner in box_corners], dtype=np.float32
        )

    # Ensure box_corners is the correct shape (4, 2)
    if box_corners.shape != (4, 2):
        raise ValueError(f"box_corners must have shape (4, 2), got {box_corners.shape}")

    half_size_x = size_x / 2
    half_size_y = size_y / 2
    # Define 3D object points in the same order as image points
    # Order: [top-left, top-right, bottom-right, bottom-left]
    unit_square = np.array(
        [
            [-half_size_x, -half_size_y, 0],  # top-left
            [half_size_x, -half_size_y, 0],  # top-right
            [half_size_x, half_size_y, 0],  # bottom-right
            [-half_size_x, half_size_y, 0],  # bottom-left
        ],
        dtype=np.float32,
    )

    # Call the fast square solver
    success, rvec, tvec = cv2.solvePnP(
        unit_square,
        box_corners,
        camera_matrix,
        dist_coeff,
        flags=cv2.SOLVEPNP_IPPE,
        useExtrinsicGuess=False,
    )

    if not success:
        raise RuntimeError("solvePnP failed")

    R, _ = cv2.Rodrigues(rvec)
    return cast(NDArray[np.float64], R), cast(NDArray[np.float64], tvec)
