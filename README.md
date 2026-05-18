<div align="center">
  <h1>TRAVEL: Traversable Ground and Above-Ground Object Segmentation</h1>
  <h3>Best Paper Award winner from RA-L 2022</h3>

  <a href="https://github.com/url-kaist/TRAVEL"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
  <a href="https://pypi.org/project/travel-seg/"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
  <a href="https://github.com/url-kaist/TRAVEL/tree/main/ros"><img src="https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-blue" /></a>
  <a href="https://github.com/url-kaist/TRAVEL"><img src="https://img.shields.io/badge/Ubuntu-E95420?logo=ubuntu&logoColor=white" /></a>
  <a href="https://github.com/url-kaist/TRAVEL"><img src="https://img.shields.io/badge/macOS-000000?logo=apple&logoColor=white" /></a>
  <a href="https://github.com/url-kaist/TRAVEL"><img src="https://img.shields.io/badge/Windows-0078D6?logo=windows&logoColor=white" /></a>
  <br />
  <a href="https://www.youtube.com/watch?v=B3CWXAsPwzU"><img src="https://img.shields.io/badge/YouTube-FF0000.svg"/></a>
  <a href="https://www.youtube.com/watch?v=GjLxv8jRM9Y&t=19s"><img src="https://img.shields.io/badge/YouTube-FF0000.svg"/></a>
  <a href="https://ieeexplore.ieee.org/document/9794594"><img src="https://img.shields.io/badge/RA_L-9794594-004088.svg"/></a>
  <a href="https://arxiv.org/abs/2206.03190"><img src="https://img.shields.io/badge/arXiv-2206.03190-004088.svg"/></a>
  <br />
  <a href="https://github.com/url-kaist/TRAVEL/actions/workflows/ci.yml"><img src="https://github.com/url-kaist/TRAVEL/actions/workflows/ci.yml/badge.svg?branch=main" alt="CI" /></a>
  <a href="https://pypi.org/project/travel-seg/"><img src="https://img.shields.io/pypi/v/travel-seg.svg" alt="PyPI" /></a>

  <p align="center"><img src="https://user-images.githubusercontent.com/47359642/193223368-d43133ec-c231-4e50-90e0-98aa0bf3a5df.gif" alt="travel_kitti" width="95%"/></p>

  <p><strong><em>Traversable Ground and Above-Ground Object Segmentation using Graph Representation for 3D LiDAR Scans.</em></strong></p>
</div>

<p align="center">
    <strong>(May 19, 2026)</strong> pip installation is now live:
    <br/>
    <a href="https://pypi.org/project/travel-seg/"><img src="https://readme-typing-svg.demolab.com?background=0D1117&color=22C55E&font=Fira+Code&size=18&duration=2500&pause=800&center=true&vCenter=true&width=280&height=30&lines=%24+pip+install+travel-seg" alt="pip install travel-seg"/></a>
</p>

______________________________________________________________________

## :rocket: Overview

Official page of "TRAVEL: Traversable Ground and Above-Ground Object Segmentation using Graph Representation for 3D LiDAR Scans", which is accepted by RA-L with IROS'22 option.

<p align="center"><img src="https://user-images.githubusercontent.com/47359642/193215974-e0e01e73-d578-458d-992f-69069b349b89.png" alt="TRAVEL_results" width="95%"/></p>

### Keywords
Object segmentation, Traversable ground segmentation, Graph search, Autonomous navigation, LiDAR

______________________________________________________________________

## :open_file_folder: Repository Layout

The repo is split so the algorithm core can be consumed without ROS or even without C++ tooling.

```
TRAVEL/
├── cpp/
│   ├── travel/        # Pure C++ core library (header-only). No ROS.
│   │   ├── core/travel/{tgs,aos,point_types,save_labels,kitti_loader,logging}.hpp
│   │   └── core/travel/3rdparty/nanoflann*.hpp
│   └── examples/      # Standalone CLI demo (run_travel_kitti). No ROS.
├── python/            # pip-installable bindings (travel-seg).
│   ├── pyproject.toml, CMakeLists.txt
│   └── travel_seg/    # numpy-friendly Python API on top of the C++ core
└── ros/               # ROS 2 (ament_cmake) wrapper that consumes cpp/travel/.
    └── src/travel_node.cpp, launch/, config/, rviz/
```

> **ROS 2 only.** As of v1.0, the `ros/` layer is ROS 2 (ament_cmake). The previous catkin / ROS 1 wrapper lives on at tag `v0.1`. Pin to that tag for Melodic / Noetic.

______________________________________________________________________

## :test_tube: Test Env.

- Algorithm core (`cpp/`): Ubuntu 20.04+, macOS 13+ (Apple Silicon / Intel) — needs only PCL + Boost.
- ROS 2 wrapper (`ros/`): verified on **Humble** (Ubuntu 22.04) and **Jazzy** (Ubuntu 24.04).

______________________________________________________________________

## :hammer: How to Build

### Python (pip)

```
pip install travel-seg
```

Prebuilt wheels are published for **Linux x86_64 (manylinux), macOS arm64 + x86_64, and Windows x86_64** for CPython 3.8–3.13 — no system PCL / Boost install required since v1.1.

For development from a checkout:

```
# Ubuntu deps:  sudo apt install build-essential cmake libeigen3-dev
# macOS deps:   brew install cmake eigen
# Windows deps: vcpkg install eigen3:x64-windows

git clone https://github.com/url-kaist/TRAVEL.git
cd TRAVEL
pip install -e python/   # editable; rebuilds C++ on next import if changed
```

Quick start:

```python
import numpy as np, travel_seg as ts
points = np.fromfile("0000.bin", dtype=np.float32).reshape(-1, 4)
result = ts.segment(points)            # SegmentResult
ground_pts = points[result.ground_mask]
```

See `python/README.md` for the full API.

### Core library + standalone example (no ROS)

```
# Dependencies
# Ubuntu:  sudo apt install cmake libeigen3-dev libboost-all-dev libpcl-dev
# macOS:   brew install cmake eigen boost pcl

cmake -S cpp/travel -B build -DTRAVEL_BUILD_EXAMPLES=ON
cmake --build build -j

# Run the demo on a KITTI sequence
./build/examples/run_travel_kitti /path/to/kitti/sequences/00 0 /tmp/travel_out
```

### ROS 2 wrapper

```
# Dependencies (Humble / Ubuntu 22.04; Jazzy / 24.04 substitutes 'humble' below)
sudo apt install build-essential cmake libeigen3-dev libpcl-dev \
                 libboost-system-dev libboost-filesystem-dev mpi-default-dev \
                 ros-humble-pcl-conversions

mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/url-kaist/TRAVEL.git
cd ..
source /opt/ros/humble/setup.bash
colcon build --packages-select travel_ros
```

Verified in CI on Humble (Ubuntu 22.04) and Jazzy (Ubuntu 24.04) via the
official `ros:humble` / `ros:jazzy` Docker images.

______________________________________________________________________

## :arrow_forward: How to Run TRAVEL

```
source install/setup.bash
ros2 launch travel_ros travel_run.launch.py input_topic:=/your_lidar_topic
```

The node subscribes to `~/input` (sensor_msgs/PointCloud2) and publishes
`~/ground`, `~/nonground`, `~/labeled`. All algorithm parameters are
declared as ROS 2 node parameters; see `ros/config/kitti_params.yaml`.

______________________________________________________________________

## :wrench: On your setting

1. Include `travel/tgs.hpp` and `travel/aos.hpp` from the `cpp/travel/core` include path.
2. Initialize `travel::TravelGroundSeg<PointT>` and `travel::ObjectCluster<PointT>`.
3. Use `setParams()` on each class to configure.
4. Use `TravelGroundSeg::estimateGround()` for traversable ground segmentation.
5. Use `ObjectCluster::segmentObjects()` for above-ground object segmentation.
6. Logging in the core is routed through `TRAVEL_LOG_*` macros (in `travel/logging.hpp`). Define `TRAVEL_USE_ROS_LOGGING` at compile time to dispatch to ROS_INFO/WARN/ERROR; otherwise output goes to stdout/stderr.

______________________________________________________________________

## :link: Related

* `pip install travel-seg` is now supported via `python/`. See `python/README.md`.
* For the previous third-party Python wrapper, see https://github.com/darrenjkt/TRAVEL. Thank you Darren :)

______________________________________________________________________

## :page_facing_up: Citation

If our research has been helpful, please cite the below papers:

```
@ARTICLE{oh2022travel,  
    author={Oh, Minho and Jung, Euigon and Lim, Hyungtae and Song, Wonho and Hu, Sumin and Lee, Eungchang Mason and Park, Junghee and Kim, Jaekyung and Lee, Jangwoo and Myung, Hyun},  
    journal={IEEE Robotics and Automation Letters},   
    title={TRAVEL: Traversable Ground and Above-Ground Object Segmentation Using Graph Representation of 3D LiDAR Scans},   
    volume={7},  
    number={3},  
    pages={7255-7262},  
    year={2022},
    }
```
```
@article{lim2021patchwork,
    title={Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor},
    author={Lim, Hyungtae and Minho, Oh and Myung, Hyun},
    journal={IEEE Robot. Autom. Lett.},
    volume={6},
    number={4},
    pages={6458--6465},
    year={2021},
    }
```
```
@article{lim2021erasor,
    title={ERASOR: Egocentric Ratio of Pseudo Occupancy-Based Dynamic Object Removal for Static 3D Point Cloud Map Building},
    author={Lim, Hyungtae and Hwang, Sungwon and Myung, Hyun},
    journal={IEEE Robotics and Automation Letters},
    volume={6},
    number={2},
    pages={2272--2279},
    year={2021},
    publisher={IEEE}
    }
```
