"""travel_seg — Python bindings for the TRAVEL ground / object segmenter.

Quick start::

    import numpy as np
    import travel_seg as ts

    points = np.fromfile("0000.bin", dtype=np.float32).reshape(-1, 4)
    result = ts.segment(points)

    ground_pts = points[result.ground_mask]
    for cluster_id in np.unique(result.instance_labels):
        if cluster_id == 0:
            continue
        cluster_pts = points[result.instance_labels == cluster_id]
        ...
"""

from ._api import ObjectCluster, TravelGroundSeg, segment
from ._config import GroundSegConfig, ObjectClusterConfig, SegmentResult

__version__ = "0.1.0"

__all__ = [
    "GroundSegConfig",
    "ObjectClusterConfig",
    "SegmentResult",
    "TravelGroundSeg",
    "ObjectCluster",
    "segment",
    "__version__",
]
