# TRAVEL for RA-L'22 w/ IROS Option 
# Best Paper Award winner from RA-L 2022
Official page of "TRAVEL: Traversable Ground and Above-Ground Object Segmentation using Graph Representation for 3D LiDAR Scans", which is accepted by RA-L with IROS'22 option.

<a href="https://www.youtube.com/watch?v=B3CWXAsPwzU"><img src="https://img.shields.io/badge/YouTube-FF0000.svg"/></a>
<a href="https://www.youtube.com/watch?v=GjLxv8jRM9Y&t=19s"><img src="https://img.shields.io/badge/YouTube-FF0000.svg"/></a>
<a href="https://ieeexplore.ieee.org/document/9794594"><img src="https://img.shields.io/badge/RA_L-9794594-004088.svg"/></a>
<a href="https://arxiv.org/abs/2206.03190"><img src="https://img.shields.io/badge/arXiv-2206.03190-004088.svg"/></a>

## Demo
![travel_kitti](https://user-images.githubusercontent.com/47359642/193223368-d43133ec-c231-4e50-90e0-98aa0bf3a5df.gif)
![TRAVEL_results](https://user-images.githubusercontent.com/47359642/193215974-e0e01e73-d578-458d-992f-69069b349b89.png)

### Keywords
Object segmentation, Traversable ground segmentation, Graph search, Autonomous navigation, LiDAR


## Repository Layout

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
└── ros/               # ROS1 (catkin) wrapper that consumes cpp/travel/.
    └── src/main.cpp, msg/, launch/, config/, rviz/
```

## Test Env.
- Algorithm core (`cpp/`): Ubuntu 20.04+, macOS 13+ (Apple Silicon / Intel) — needs only PCL + Boost.
- ROS wrapper (`ros/`): Ubuntu 18.04 / ROS Melodic (original target). Newer combos (Ubuntu 20.04 / Noetic) should work but are not regression-tested.

## How to Build

### Python (pip)

```
# Ubuntu deps:  sudo apt install build-essential cmake libeigen3-dev libpcl-dev libboost-system-dev libboost-filesystem-dev
# macOS deps:   brew install cmake eigen boost pcl

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

### ROS1 wrapper

The catkin package now lives under `ros/`. Cloning the repo into a catkin workspace works as-is, because catkin discovers `ros/package.xml` recursively and the `ros/CMakeLists.txt` pulls in the C++ core via `add_subdirectory(../cpp/travel)`.

```
# Dependencies (Noetic / Ubuntu 20.04 shown — Melodic / 18.04 also works)
sudo apt install cmake libeigen3-dev libboost-all-dev
sudo apt-get install ros-noetic-pcl-ros ros-noetic-pcl-conversions

mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/url-kaist/TRAVEL.git
cd ..
catkin_make
```

Verified: Ubuntu 20.04 + ROS Noetic via `osrf/ros:noetic-desktop-full` Docker image.

## How to Run TRAVEL

```
roslaunch travel travel_run.launch
```

## On your setting

1. Include `travel/tgs.hpp` and `travel/aos.hpp` from the `cpp/travel/core` include path.
2. Initialize `travel::TravelGroundSeg<PointT>` and `travel::ObjectCluster<PointT>`.
3. Use `setParams()` on each class to configure.
4. Use `TravelGroundSeg::estimateGround()` for traversable ground segmentation.
5. Use `ObjectCluster::segmentObjects()` for above-ground object segmentation.
6. Logging in the core is routed through `TRAVEL_LOG_*` macros (in `travel/logging.hpp`). Define `TRAVEL_USE_ROS_LOGGING` at compile time to dispatch to ROS_INFO/WARN/ERROR; otherwise output goes to stdout/stderr.

* `pip install travel-seg` is now supported via `python/`. See `python/README.md`.
* For the previous third-party Python wrapper, see https://github.com/darrenjkt/TRAVEL. Thank you Darren :)

## Citation
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
