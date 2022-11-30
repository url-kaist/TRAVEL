#ifndef TRAVEL_UTILS_H
#define TRAVEL_UTILS_H

#include <vector>
#include <map>
#include <iostream>
#include <sstream>
#include <signal.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include "utils/nanoflann.hpp"
#include "utils/nanoflann_utils.hpp"

#define INVALID_IDX -1
struct PointXYZILID
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float    intensity;                 ///< laser intensity reading
  uint16_t label;                     ///< point label
  uint16_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILID,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, label, label)
                                  (uint16_t, id, id))

using PointT = PointXYZILID;
using num_t = float;

void PointXYZILID2XYZI(pcl::PointCloud<PointXYZILID>& src,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr dst){
  dst->points.clear();
  for (const auto &pt: src.points){
    pcl::PointXYZI pt_xyzi;
    pt_xyzi.x = pt.x;
    pt_xyzi.y = pt.y;
    pt_xyzi.z = pt.z;
    pt_xyzi.intensity = pt.intensity;
    dst->points.push_back(pt_xyzi);
  }
}

template <typename PointType>
void saveLabels(const std::string abs_dir, const int frame_num, const pcl::PointCloud<PointType> &cloud_in,
                                             const pcl::PointCloud<PointType> &labeled_pc) {
    // Save labels as a .label file
    // It is relevant to 3DUIS benchmark
    // https://codalab.lisn.upsaclay.fr/competitions/2183?secret_key=4763e3d2-1f22-45e6-803a-a862528426d2
    // Labels are set in the intensity of `labeled_pc`, which is larger than 0. i.e. > 0
    const float SQR_EPSILON = 0.00001;

    int num_cloud_in = cloud_in.points.size();
    std::vector<uint32_t> labels(num_cloud_in, 0); // 0: not interested

    int N = labeled_pc.points.size();
    PointCloud<num_t> cloud;
    cloud.pts.resize(N);
    for (size_t i = 0; i < N; i++)
    {
        cloud.pts[i].x = labeled_pc.points[i].x;
        cloud.pts[i].y = labeled_pc.points[i].y;
        cloud.pts[i].z = labeled_pc.points[i].z;
    }

    // construct a kd-tree index:
    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t>>,
    PointCloud<num_t>, 3 /* dim */
                       >;

    my_kd_tree_t index(3 /*dim*/, cloud, {10 /* max leaf */});

    int num_valid = 0;
    for (int j = 0; j < cloud_in.points.size(); ++j) {
        const auto query_pcl = cloud_in.points[j];
        const num_t query_pt[3] = {query_pcl.x, query_pcl.y, query_pcl.z};
        {
            size_t num_results = 1;
            std::vector<uint32_t> ret_index(num_results);
            std::vector<num_t> out_dist_sqr(num_results);

            num_results = index.knnSearch(
                    &query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

            // In case of less points in the tree than requested:
            ret_index.resize(num_results);
            out_dist_sqr.resize(num_results);
            if(out_dist_sqr[0] < SQR_EPSILON) { // it is the same point!
                labels[j] = labeled_pc.points[ret_index[0]].intensity;
                ++num_valid;
            }
        }
    }
    // Must be equal to the # of above-ground points
    std::cout << "# of valid points: " << num_valid << std::endl;

     //  To follow the KITTI format, # of zeros are set to 6
    const int NUM_ZEROS = 6;

    std::string count_str = std::to_string(frame_num);
    std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
    std::string abs_label_path = abs_dir + "/" + count_str_padded + ".label";

    // Semantics are just set to be zero
    for (int i = 0; i < labels.size(); ++i) {
        uint32_t shifted = labels[i] << 16;
        labels[i] = shifted;
    }

    std::cout << "\033[1;32m" << abs_label_path << "\033[0m" << std::endl;
    std::ofstream output_file(abs_label_path, std::ios::out | std::ios::binary);
    output_file.write(reinterpret_cast<char*>(&labels[0]), num_cloud_in * sizeof(uint32_t));
}

#endif
