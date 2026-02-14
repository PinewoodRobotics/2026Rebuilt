import numpy as np
import pytest

from backend.python.pos_extrapolator.filters.gate.mahalanobis import (
    mahalanobis_distance,
    percent_confidence,
)


def test_mahalanobis_identity_matches_euclidean():
    x = np.array([3.0, 4.0])
    xhat = np.array([0.0, 0.0])
    R = np.eye(2)
    d = mahalanobis_distance(x, xhat, R)
    assert d == pytest.approx(5.0)


def test_mahalanobis_squared_option():
    x = np.array([3.0, 4.0])
    xhat = np.array([0.0, 0.0])
    R = np.eye(2)
    d2 = mahalanobis_distance(x, xhat, R, squared=True)
    assert d2 == pytest.approx(25.0)


def test_mahalanobis_raises_on_shape_mismatch():
    x = np.array([1.0, 2.0, 3.0])
    xhat = np.array([1.0, 2.0])
    R = np.eye(2)
    with pytest.raises(ValueError):
        _ = mahalanobis_distance(x, xhat, R)


def test_mahalanobis_raises_on_bad_R_shape():
    x = np.array([1.0, 2.0])
    xhat = np.array([0.0, 0.0])
    R = np.eye(3)
    with pytest.raises(ValueError):
        _ = mahalanobis_distance(x, xhat, R)


def test_mahalanobis_singular_R_falls_back_to_pinvh():
    # Singular covariance: second dimension has zero variance.
    x = np.array([1.0, 0.0])
    xhat = np.array([0.0, 0.0])
    R = np.array([[1.0, 0.0], [0.0, 0.0]])
    d2 = mahalanobis_distance(x, xhat, R, squared=True)
    assert d2 == pytest.approx(1.0)


def test_percent_confidence_extremes():
    assert percent_confidence(0.0, measurement_dim=2) == pytest.approx(100.0)
    assert percent_confidence(1e9, measurement_dim=2) == pytest.approx(0.0, abs=1e-6)
