"""Python mirror of cpp/examples/run_travel_kitti.cpp.

Reads a single KITTI velodyne scan, runs ground + object segmentation, and
prints summary statistics. Optionally writes per-class .npy files for
inspection.

Usage:
    python run_kitti.py <kitti_seq_dir> [frame_index] [output_dir]

    <kitti_seq_dir>  directory containing velodyne/<XXXXXX>.bin
    [frame_index]    0-based frame index (default 0)
    [output_dir]     if given, writes <prefix>_ground.npy / _nonground.npy /
                     _labels.npy (one int32 label per input point)
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

import travel_seg as ts


def load_kitti_bin(seq_dir: Path, frame_idx: int) -> np.ndarray:
    """Load a single KITTI velodyne_points/data .bin file as float32 (N, 4)."""
    path = seq_dir / "velodyne" / f"{frame_idx:06d}.bin"
    if not path.exists():
        raise FileNotFoundError(f"velodyne frame not found: {path}")
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__, file=sys.stderr)
        return 1

    seq_dir    = Path(argv[1])
    frame_idx  = int(argv[2]) if len(argv) > 2 else 0
    output_dir = Path(argv[3]) if len(argv) > 3 else None

    points = load_kitti_bin(seq_dir, frame_idx)
    print(f"Loaded {points.shape[0]} points from {seq_dir.name} frame {frame_idx}")

    t0 = time.perf_counter()
    result = ts.segment(points[:, :3])
    wall = time.perf_counter() - t0

    ground_n    = int(result.ground_mask.sum())
    nonground_n = int((~result.ground_mask).sum())
    cluster_ids = np.unique(result.instance_labels)
    cluster_n   = int((cluster_ids > 0).sum())
    print(
        f"  ground: {ground_n} | nonground: {nonground_n} | "
        f"clusters: {cluster_n} | tgs: {result.elapsed_seconds*1000:.1f} ms | "
        f"wall: {wall*1000:.1f} ms"
    )

    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)
        prefix = output_dir / f"{frame_idx:06d}"
        np.save(prefix.with_suffix(".ground.npy"),    points[result.ground_mask])
        np.save(prefix.with_suffix(".nonground.npy"), points[~result.ground_mask])
        np.save(prefix.with_suffix(".labels.npy"),    result.instance_labels)
        print(f"  wrote {prefix.parent}/{prefix.name}.{{ground,nonground,labels}}.npy")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
