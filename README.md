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

### To Do 
* This archive is still being updated. Up until the IROS'22 presentation, I will update the final version.
- [ ] runnable node on general setting.
- [ ] Add the application codes
- [ ] Update for docker users

## Contributors
- Minho Oh & Euigon Jung
- email: 
    - Minho Oh: minho.oh@kaist.ac.kr
    - Euigon Jung: euigon94@gmail.com
- Advised from Hyungtae Lim and Hyun Myung @ KAIST

## Test Env.
- Ubuntu 18.04 LTS
- ROS Melodic

## How to Build
1. Dependencies
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
   
## How to Run In KITTI 

```bash
roslaunch travel run_kitti.launch data_dir:=${ABS_DIRECTORY_OF_KITTI} seq:=${SEQUENCE_ID}
# for example,
# then, the bin files in `/home/shapelim/qbench/kitti/00/velodyne` are loaded in the ascending order 
roslaunch travel run_kitti.launch data_dir:=/home/shapelim/qbench/kitti seq:=00
```

## Download our dataset
- rough_terrain_1 (24.5GB) and rough_terrain_2 (15.4GB) are our rough terrain dataset.
    
    ```
    wget https://urserver.kaist.ac.kr/publicdata/TRAVEL_22/rough_terrain_1.bag
    wget https://urserver.kaist.ac.kr/publicdata/TRAVEL_22/rough_terrain_2.bag
    ```
    
## How to Run TRAVEL
- RUN!
    ```
    roslaunch travel run.launch
    ```

## On your setting.
1. Include two header files in your source. "tgs.hpp" & "aos.hpp"
2. Initialize "travel::TravelGroundSeg<PointT>" and "travel::ObjectCluster<PointT>"
3. Use the "setParams()" function in each class to set the parameters.
4. Use "travel::TravelGroundSeg.estimateGround()" function for traversable ground segmentation
5. Use "travel::ObjectCluster.segmentObjects()" function for above-ground object segmentation
* I will upload an example ros node that subscribes to sensor data.
* If you want to use TRAVEL with python code, then visit here (https://github.com/darrenjkt/TRAVEL). Thank you Darren :)

## Aplications
- Local terrain map.

- Global traversable map.

- Global static traversable map by using TRAVEL with [ERASOR](https://github.com/LimHyungTae/ERASOR)
![traversable_map](https://user-images.githubusercontent.com/47359642/193323305-288da2c8-5083-45e8-8781-815ae8ee592d.png)

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
