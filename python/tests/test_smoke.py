"""Smoke tests for travel_seg.

Goal: prove the binding builds, imports, and runs end-to-end on synthetic
data — not to validate algorithmic quality. Numerical correctness on KITTI
is covered by examples/run_kitti.py against real data.
"""

from __future__ import annotations

import numpy as np
import pytest

import travel_seg as ts


def _synthetic_scene(n_ground: int = 40_000,
                     n_objects: int = 8_000,
                     seed: int = 0) -> np.ndarray:
    """A flat ground plane plus a few box-shaped clusters above it.

    Range is roughly 5..40 m so it lands inside the default
    ``GroundSegConfig`` / ``ObjectClusterConfig`` ranges.
    """
    rng = np.random.default_rng(seed)

    ground = np.zeros((n_ground, 3), dtype=np.float32)
    ground[:, 0] = rng.uniform(-30.0, 30.0, n_ground)
    ground[:, 1] = rng.uniform(-30.0, 30.0, n_ground)
    ground[:, 2] = rng.normal(0.0, 0.02, n_ground)

    object_pts = []
    centers = [(8, 0, 1.0), (-6, 4, 0.8), (3, -10, 1.2), (15, 5, 1.5)]
    per_box = n_objects // len(centers)
    for cx, cy, cz in centers:
        box = np.zeros((per_box, 3), dtype=np.float32)
        box[:, 0] = cx + rng.uniform(-0.6, 0.6, per_box)
        box[:, 1] = cy + rng.uniform(-0.6, 0.6, per_box)
        box[:, 2] = rng.uniform(0.2, cz + 0.5, per_box)
        object_pts.append(box)
    objects = np.concatenate(object_pts, axis=0)

    pts = np.concatenate([ground, objects], axis=0)
    rng.shuffle(pts)
    return pts


def test_module_metadata():
    assert hasattr(ts, "__version__")
    assert callable(ts.segment)


def test_segment_returns_correct_shapes():
    points = _synthetic_scene()
    result = ts.segment(points)

    assert isinstance(result, ts.SegmentResult)
    assert result.ground_mask.shape == (points.shape[0],)
    assert result.ground_mask.dtype == np.bool_
    assert result.instance_labels.shape == (points.shape[0],)
    assert result.instance_labels.dtype == np.int32
    assert isinstance(result.elapsed_seconds, float)
    assert result.elapsed_seconds >= 0.0


def test_ground_segmentation_finds_ground_points():
    points = _synthetic_scene()
    result = ts.segment(points)

    # The synthetic ground plane has |z| ~ 0; the boxes are at z >= 0.2. So
    # most low-z points should land in ground_mask, and most high-z points
    # should not. We don't require a perfect classifier here, just that the
    # algorithm is producing sensibly correlated output.
    low_z = points[:, 2] < 0.05
    high_z = points[:, 2] > 0.5

    ground_recall_lowz = result.ground_mask[low_z].mean() if low_z.any() else 0.0
    ground_in_highz    = result.ground_mask[high_z].mean() if high_z.any() else 0.0

    assert ground_recall_lowz > 0.5, (
        f"expected most low-z points to be ground, got {ground_recall_lowz:.2%}")
    assert ground_in_highz < 0.5, (
        f"expected most high-z points to not be ground, got {ground_in_highz:.2%}")


def test_object_clustering_produces_multiple_clusters():
    points = _synthetic_scene()
    result = ts.segment(points)

    cluster_ids = np.unique(result.instance_labels)
    nonzero = cluster_ids[cluster_ids > 0]
    # We seeded 4 well-separated boxes; expect at least a couple of clusters.
    assert len(nonzero) >= 2, (
        f"expected multiple clusters from 4 boxes, got ids {cluster_ids}")


def test_xyz_and_xyzi_inputs_both_accepted():
    points_xyz = _synthetic_scene()
    intensity  = np.zeros((points_xyz.shape[0], 1), dtype=np.float32)
    points_xyzi = np.concatenate([points_xyz, intensity], axis=1)

    r_xyz  = ts.segment(points_xyz)
    r_xyzi = ts.segment(points_xyzi)

    # Ground segmentation is deterministic; cluster id assignment is not
    # (AOS::labelPointcloud uses std::random_device to shuffle ids), so we
    # check structural equivalence instead of bit-identity for the labels.
    np.testing.assert_array_equal(r_xyz.ground_mask, r_xyzi.ground_mask)

    labeled_xyz  = r_xyz.instance_labels  > 0
    labeled_xyzi = r_xyzi.instance_labels > 0
    np.testing.assert_array_equal(labeled_xyz, labeled_xyzi)
    assert len(np.unique(r_xyz.instance_labels[labeled_xyz])) == \
           len(np.unique(r_xyzi.instance_labels[labeled_xyzi]))


def test_class_api_matches_convenience_function():
    points = _synthetic_scene()

    tgs = ts.TravelGroundSeg()
    aos = ts.ObjectCluster()

    ground_mask, _ = tgs.estimate_ground(points)
    instance_labels = np.zeros(points.shape[0], dtype=np.int32)
    nonground = points[~ground_mask]
    if len(nonground) > 0:
        instance_labels[~ground_mask] = aos.segment_objects(nonground)

    result = ts.segment(points)
    # Ground segmentation is deterministic; cluster ids are not (see above),
    # so compare ground equality and labeled-vs-unlabeled membership only.
    np.testing.assert_array_equal(ground_mask, result.ground_mask)
    np.testing.assert_array_equal(instance_labels > 0, result.instance_labels > 0)


def test_invalid_input_shape_raises():
    with pytest.raises(ValueError):
        ts.segment(np.zeros(100, dtype=np.float32))
    with pytest.raises(ValueError):
        ts.segment(np.zeros((100, 5), dtype=np.float32))
