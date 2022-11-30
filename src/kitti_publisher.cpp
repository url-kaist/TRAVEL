//
// Created by Hyungtae Lim on 6/23/21.
//

// For disable PCL complile lib, to use PointXYZILID
#define PCL_NO_PRECOMPILE
#include "travel/node.h"
#include "utils/kitti_loader.hpp"
#include "utils/utils.hpp"

#include "travel/aos.hpp"
#include "travel/tgs.hpp"

using PointType = pcl::PointXYZI;
using namespace std;

ros::Publisher NodePublisher;

string data_dir;
string seq;

void callbackSignalHandler(int signum) {
    cout << "Caught Ctrl + c " << endl;
    // Terminate program
    exit(signum);
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "Ros-Kitti-Publisher");

    int kitti_hz;
    ros::NodeHandle nh;
    std::string node_topic;
    nh.param<string>("/node_topic" , node_topic, "/node");
    nh.param<string>("/data_dir", data_dir, "/");
    nh.param<string>("/seq", seq, "");
    nh.param<int>("/kitti_hz", kitti_hz, 10);

    cout << "\033[1;32m" << "Node topic: " << node_topic << "\033[0m" << endl;
    cout << "\033[1;32m" << "KITTI data directory: " << data_dir << "\033[0m" << endl;
    cout << "\033[1;32m" << "Sequence: " << seq << "\033[0m" << endl;

    ros::Rate r(kitti_hz);
    ros::Publisher NodePublisher = nh.advertise<travel::node>(node_topic, 100, true);


    std::string data_path = data_dir + "/" + seq;
    KittiLoader loader(data_path);
    int      N = loader.size();

    signal(SIGINT, callbackSignalHandler);
    cout << "\033[1;32m[Kitti Publisher] Total " << N << " clouds are loaded\033[0m" << endl;
    for (int n = 0; n < N; ++n) {
        cout << n << "th node is published!" << endl;
        pcl::PointCloud<pcl::PointXYZI> pc_curr; // (new pcl::PointCloud<PointType>);

        pc_curr = *loader.cloud(n);
        std::cout << "Complete load!" << std::endl;
        travel::node node;

        node.lidar = cloud2msg(pc_curr);
        node.header = node.lidar.header;
        node.header.seq = n;
        NodePublisher.publish(node);
        r.sleep();
    }
    return 0;
}