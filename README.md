# TRAVEL for RA-L'22 w/ IROS Option
Official page of "TRAVEL: Traversable Ground and Above-Ground Object Segmentation using Graph Representation for 3D LiDAR Scans", which is accepted by RA-L with IROS'22 option.

<a href="https://www.youtube.com/watch?v=B3CWXAsPwzU"><img src="https://img.shields.io/badge/YouTube-FF0000.svg"/></a>
<a href="https://www.youtube.com/watch?v=GjLxv8jRM9Y&t=19s"><img src="https://img.shields.io/badge/YouTube-FF0000.svg"/></a>
<a href="https://ieeexplore.ieee.org/document/9794594"><img src="https://img.shields.io/badge/RA_L-9794594-004088.svg"/></a>
<a href="https://arxiv.org/abs/2206.03190"><img src="https://img.shields.io/badge/arXiv-2206.03190-004088.svg"/></a>

### Keywords
Object segmentation, Traversable ground segmentation, Graph search, Autonomous navigation, LiDAR

## Demo
![travel_kitti](https://user-images.githubusercontent.com/47359642/193223368-d43133ec-c231-4e50-90e0-98aa0bf3a5df.gif)
![TRAVEL_results](https://user-images.githubusercontent.com/47359642/193215974-e0e01e73-d578-458d-992f-69069b349b89.png)

## Contributors
- Minho Oh & Euigon Jung
- email: 
    - Minh Oh: minho.oh@kaist.ac.kr
    - Euigon Jung: euigon94@kaist.ac.kr

## Test Env.
** Ubuntu 18.04 LTS
** ROS Melodic

## How to Build
1. Dedendencies
    ```
    sudo apt install cmake libeigen3-dev libboost-all-dev
    sudo apt-get install ros-melodic-jsk-recognition
    sudo apt-get install ros-melodic-jsk-common-msgs
    sudo apt-get install ros-melodic-jsk-rviz-plugins
    ```
2. Build
    ```
    mkdir -p catkin_ws/src/
    cd catkin_ws/src/
    git clone https://github.com/url-kaist/TRAVEL.git
    ../
    catkin_make
    ```

## Download our dataset
- rough_terrain_1 (24.5GB) and rough_terrain_2 (15.4GB) are our rough terrain dataset.
    
    ```
    wget https://urserver.kaist.ac.kr/publicdata/TRAVEL_22/rough_terrain_1.bag
    wget https://urserver.kaist.ac.kr/publicdata/TRAVEL_22/rough_terrain_2.bag
    ```
    
## How to Run TRAVEL

## On your setting.
###

## Aplications

## Citation
If our research has been helpful, please cite the below papers:


