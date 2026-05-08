"""User-facing Python API for travel_seg.

Wraps the compiled ``_travel_seg`` module in idiomatic Python classes that
take dataclass configs and return numpy arrays (instead of the C++ idiom of
output references and PCL point clouds).
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from ._config import GroundSegConfig, ObjectClusterConfig, SegmentResult


def _backend():
    # Lazy import so a missing/failed binding produces a useful traceback at
    # the call site instead of at module import time.
    try:
        from . import _travel_seg as backend
    except ImportError:
        import _travel_seg as backend  # editable / non-installed fallback
    return backend


def _ensure_xyz_or_xyzi_float32(points: np.ndarray) -> np.ndarray:
    arr = np.asarray(points)
    if arr.ndim != 2:
        raise ValueError(f"expected 2D array, got {arr.ndim}D with shape {arr.shape}")
    if arr.shape[1] not in (3, 4):
        raise ValueError(
            f"expected (N, 3) for XYZ or (N, 4) for XYZI, got shape {arr.shape}"
        )
    if arr.dtype != np.float32:
        arr = arr.astype(np.float32, copy=False)
    if not arr.flags["C_CONTIGUOUS"]:
        arr = np.ascontiguousarray(arr)
    return arr


class TravelGroundSeg:
    """Traversable ground segmentation.

    Reuses internal state across calls, so creating the instance once and
    calling ``estimate_ground`` per frame is faster than going through
    :func:`segment` (which builds a fresh instance every call).
    """

    def __init__(self, config: Optional[GroundSegConfig] = None):
        self._config = config if config is not None else GroundSegConfig()
        self._impl = _backend()._TravelGroundSeg()
        self._apply_config()

    def _apply_config(self) -> None:
        c = self._config
        self._impl.set_params(
            max_range=c.max_range,
            min_range=c.min_range,
            resolution=c.resolution,
            num_iter=c.num_iter,
            num_lpr=c.num_lpr,
            num_min_pts=c.num_min_pts,
            th_seeds=c.th_seeds,
            th_dist=c.th_dist,
            th_outlier=c.th_outlier,
            th_normal=c.th_normal,
            th_weight=c.th_weight,
            th_lcc_normal=c.th_lcc_normal,
            th_lcc_planar=c.th_lcc_planar,
            th_obstacle=c.th_obstacle,
            refine_mode=c.refine_mode,
        )

    @property
    def config(self) -> GroundSegConfig:
        return self._config

    @config.setter
    def config(self, value: GroundSegConfig) -> None:
        self._config = value
        self._apply_config()

    def estimate_ground(self, points: np.ndarray) -> Tuple[np.ndarray, float]:
        """Run TGS on a single frame.

        Parameters
        ----------
        points:
            ``float32`` numpy array of shape ``(N, 3)`` (XYZ) or
            ``(N, 4)`` (XYZI). Mixed dtypes are silently cast.

        Returns
        -------
        (ground_mask, elapsed_seconds):
            ``ground_mask`` is a ``bool`` array of length ``N``; ``True``
            wherever the corresponding input point was classified as ground.
            ``elapsed_seconds`` is wall-clock time spent inside the C++
            algorithm.
        """
        return self._impl.estimate_ground(_ensure_xyz_or_xyzi_float32(points))


class ObjectCluster:
    """Above-ground object segmentation (instance clustering)."""

    def __init__(self, config: Optional[ObjectClusterConfig] = None):
        self._config = config if config is not None else ObjectClusterConfig()
        self._impl = _backend()._ObjectCluster()
        self._apply_config()

    def _apply_config(self) -> None:
        c = self._config
        self._impl.set_params(
            vert_scan=c.vert_scan,
            horz_scan=c.horz_scan,
            min_range=c.min_range,
            max_range=c.max_range,
            min_vert_angle=c.min_vert_angle,
            max_vert_angle=c.max_vert_angle,
            horz_merge_thres=c.horz_merge_thres,
            vert_merge_thres=c.vert_merge_thres,
            vert_scan_size=c.vert_scan_size,
            horz_scan_size=c.horz_scan_size,
            horz_extension_size=c.horz_extension_size,
            horz_skip_size=c.horz_skip_size,
            downsample=c.downsample,
            min_cluster_size=c.min_cluster_size,
            max_cluster_size=c.max_cluster_size,
        )
        if c.seed is None:
            self._impl.clear_seed()
        else:
            self._impl.set_seed(int(c.seed))

    @property
    def config(self) -> ObjectClusterConfig:
        return self._config

    @config.setter
    def config(self, value: ObjectClusterConfig) -> None:
        self._config = value
        self._apply_config()

    def segment_objects(self, points: np.ndarray) -> np.ndarray:
        """Cluster the given (typically non-ground) points.

        Parameters
        ----------
        points:
            ``float32`` numpy array of shape ``(M, 3)`` (XYZ) or
            ``(M, 4)`` (XYZI).

        Returns
        -------
        instance_labels:
            ``int32`` array of length ``M``. ``0`` means the point was not
            assigned to any cluster (e.g. dropped by AOS range /
            downsample / cluster-size filters). Positive ids are unique
            within this single frame.
        """
        return self._impl.segment_objects(_ensure_xyz_or_xyzi_float32(points))


def segment(
    points: np.ndarray,
    ground_config: Optional[GroundSegConfig] = None,
    object_config: Optional[ObjectClusterConfig] = None,
) -> SegmentResult:
    """One-shot TGS + AOS over a single frame.

    Equivalent to::

        tgs = TravelGroundSeg(ground_config)
        aos = ObjectCluster(object_config)
        ground_mask, t = tgs.estimate_ground(points)
        labels = np.zeros(len(points), dtype=np.int32)
        labels[~ground_mask] = aos.segment_objects(points[~ground_mask])

    For per-frame loops, prefer constructing :class:`TravelGroundSeg` and
    :class:`ObjectCluster` once and reusing them; this convenience function
    rebuilds both on every call.
    """
    points = _ensure_xyz_or_xyzi_float32(points)

    tgs = TravelGroundSeg(ground_config)
    aos = ObjectCluster(object_config)

    ground_mask, elapsed = tgs.estimate_ground(points)

    instance_labels = np.zeros(points.shape[0], dtype=np.int32)
    nonground_idx = np.flatnonzero(~ground_mask)
    if nonground_idx.size > 0:
        nonground_labels = aos.segment_objects(points[nonground_idx])
        instance_labels[nonground_idx] = nonground_labels

    return SegmentResult(
        ground_mask=ground_mask,
        instance_labels=instance_labels,
        elapsed_seconds=elapsed,
    )
