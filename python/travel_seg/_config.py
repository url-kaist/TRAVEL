"""Configuration dataclasses and result types for travel_seg.

Defaults mirror ``ros/config/kitti_params.yaml`` so the Python API gives
sensible behaviour on KITTI-style 64-channel scans out of the box.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class GroundSegConfig:
    """Parameters for traversable ground segmentation (TGS)."""
    max_range: float = 80.0
    min_range: float = 1.0
    resolution: float = 8.0
    num_iter: int = 3
    num_lpr: int = 5
    num_min_pts: int = 10
    th_seeds: float = 0.5
    th_dist: float = 0.125
    th_outlier: float = 0.3
    th_normal: float = 0.940
    th_weight: float = 200.0
    th_lcc_normal: float = 0.03
    th_lcc_planar: float = 0.1
    th_obstacle: float = 1.0
    refine_mode: bool = True


@dataclass
class ObjectClusterConfig:
    """Parameters for above-ground object segmentation / clustering (AOS)."""
    vert_scan: int = 64
    horz_scan: int = 4500
    min_range: float = 1.0
    max_range: float = 80.0
    min_vert_angle: float = -24.8
    max_vert_angle: float = 2.0
    horz_merge_thres: float = 0.4
    vert_merge_thres: float = 0.5
    vert_scan_size: int = 3
    horz_scan_size: int = 5
    horz_extension_size: int = 5
    horz_skip_size: int = 5
    downsample: int = 1
    min_cluster_size: int = 10
    max_cluster_size: int = 30000


@dataclass
class SegmentResult:
    """Result of a one-shot ``travel_seg.segment(points)`` call.

    Attributes
    ----------
    ground_mask:
        Boolean array of length ``N`` (= number of input points). ``True``
        where the corresponding input point was classified as ground.
    instance_labels:
        ``int32`` array of length ``N``. ``0`` means the point was either
        ground or fell outside any cluster (e.g. trimmed by AOS range /
        downsample / cluster-size filters). ``1..K`` are per-cluster ids
        within this single frame; ids are not consistent across frames.
    elapsed_seconds:
        Wall-clock time spent in the C++ ground segmentation step. Object
        clustering time is currently not reported separately.
    """
    ground_mask: np.ndarray
    instance_labels: np.ndarray
    elapsed_seconds: float
