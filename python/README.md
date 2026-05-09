# travel-seg

Python bindings for [TRAVEL](https://github.com/url-kaist/TRAVEL) — traversable ground and above-ground object segmentation for 3D LiDAR scans.

## Install

### From PyPI (recommended)

```bash
pip install travel-seg
```

Prebuilt wheels are published for **Linux x86_64 (manylinux_2_17), macOS arm64 + x86_64, and Windows x86_64** for CPython 3.8–3.13. As of v1.1, the only system dependency is Eigen — no PCL / Boost install required.

### From source (development)

```bash
# System deps (Ubuntu)
sudo apt install build-essential cmake libeigen3-dev

# System deps (macOS)
brew install cmake eigen

# System deps (Windows)
vcpkg install eigen3:x64-windows
# then: -DCMAKE_PREFIX_PATH="C:/vcpkg/installed/x64-windows" forwarded by
# the release workflow / pip env on Windows.

git clone https://github.com/url-kaist/TRAVEL.git
cd TRAVEL
pip install -e python/
```

## Usage

```python
import numpy as np
import travel_seg as ts

# KITTI .bin → (N, 4) float32 XYZI
points = np.fromfile("0000.bin", dtype=np.float32).reshape(-1, 4)

# One-shot
result = ts.segment(points)
ground_pts = points[result.ground_mask]
for cid in np.unique(result.instance_labels):
    if cid == 0:
        continue
    cluster_pts = points[result.instance_labels == cid]
    print(f"cluster {cid}: {len(cluster_pts)} points")

# Per-frame loops: build the segmenters once, reuse
tgs = ts.TravelGroundSeg(ts.GroundSegConfig(max_range=80.0))
aos = ts.ObjectCluster(ts.ObjectClusterConfig(vert_scan=64, horz_scan=4500))
for frame in frames:
    ground_mask, elapsed = tgs.estimate_ground(frame[:, :3])
    labels = aos.segment_objects(frame[~ground_mask, :3])
    ...
```

## API

| Symbol | Purpose |
|---|---|
| `GroundSegConfig`     | dataclass — TGS parameters; KITTI-tuned defaults |
| `ObjectClusterConfig` | dataclass — AOS parameters; KITTI-tuned defaults |
| `TravelGroundSeg(cfg)` | reusable ground segmenter; `.estimate_ground(points) -> (mask, elapsed)` |
| `ObjectCluster(cfg)`   | reusable object clusterer; `.segment_objects(points) -> labels` |
| `segment(points, ground_config=None, object_config=None)` | one-shot helper returning a `SegmentResult` |
| `SegmentResult`        | dataclass: `.ground_mask`, `.instance_labels`, `.elapsed_seconds` |

Inputs are float32 numpy arrays of shape `(N, 3)` (XYZ) or `(N, 4)` (XYZI). The intensity column is currently ignored by both TGS and AOS but accepted for convenience when reading raw KITTI bins.

Outputs are length-N numpy arrays aligned with the original input order:
- `ground_mask`: `bool`, `True` where the input point was classified as ground.
- `instance_labels`: `int32`. `0` = ground / unclassified / dropped. `1..K` = per-cluster ids within this single frame (not consistent across frames).

## Tests / examples

```bash
pip install -e python/[test]
pytest python/tests/

# Real data demo
python python/examples/run_kitti.py /path/to/kitti/sequences/00 0 /tmp/out
```

## Citation

See the top-level [README](../README.md).

## Release process

Publishing uses **PyPI Trusted Publishing (OIDC)**, so there are no API
tokens or GitHub secrets to manage. PyPI authenticates the GitHub Actions
workflow directly based on a one-time pending-publisher registration.

### One-time PyPI setup

Visit https://pypi.org/manage/account/publishing/ and add a *pending*
publisher with these values (only required before the very first release;
it auto-graduates to a regular trusted publisher after the first upload):

| Field | Value |
|---|---|
| PyPI Project Name | `travel-seg` |
| Owner | `url-kaist` |
| Repository name | `TRAVEL` |
| Workflow name | `release.yml` |
| Environment name | `pypi` |

GitHub side: create an environment named `pypi` at Settings -> Environments
(no secrets/protection rules needed unless you want manual approval).

### Per-release flow

1. Bump version in three places to keep them in sync:
   - `python/pyproject.toml` -> `[project] version`
   - `python/travel_seg/__init__.py` -> `__version__`
   - `python/travel_seg/pybind/travel_pybind.cpp` -> `m.attr("__version__")`
2. Commit, tag, push:
   ```bash
   git commit -am "release: travel-seg vX.Y.Z"
   git tag vX.Y.Z
   git push origin main vX.Y.Z
   ```
3. Trigger the **Release (PyPI)** workflow from the GitHub Actions UI.
4. Verify the install in a fresh venv:
   ```bash
   python -m venv /tmp/check && source /tmp/check/bin/activate
   pip install travel-seg==X.Y.Z
   python -c "import travel_seg as ts; import numpy as np; \
              ts.segment(np.random.uniform(-10,10,(2000,3)).astype(np.float32))"
   ```
