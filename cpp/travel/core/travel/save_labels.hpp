#ifndef TRAVEL_SAVE_LABELS_HPP
#define TRAVEL_SAVE_LABELS_HPP

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "travel/types.hpp"

#include "travel/point_types.hpp"
#include "travel/3rdparty/nanoflann.hpp"
#include "travel/3rdparty/nanoflann_utils.hpp"

template <typename PointType>
void saveLabels(const std::string abs_dir,
                const int frame_num,
                const travel::PointCloud<PointType>& cloud_in,
                const travel::PointCloud<PointType>& labeled_pc) {
    // Save labels as a .label file. Compatible with the 3DUIS benchmark
    // (https://codalab.lisn.upsaclay.fr/competitions/2183).
    // Labels are taken from the intensity of `labeled_pc` (must be > 0).
    const float SQR_EPSILON = 0.00001f;

    const int num_cloud_in = static_cast<int>(cloud_in.points.size());
    std::vector<uint32_t> labels(num_cloud_in, 0);  // 0 == not interested

    const int N = static_cast<int>(labeled_pc.points.size());
    PointCloud<num_t> cloud;
    cloud.pts.resize(N);
    for (int i = 0; i < N; i++) {
        cloud.pts[i].x = labeled_pc.points[i].x;
        cloud.pts[i].y = labeled_pc.points[i].y;
        cloud.pts[i].z = labeled_pc.points[i].z;
    }

    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t>>,
            PointCloud<num_t>, 3 /* dim */>;

    my_kd_tree_t index(3 /*dim*/, cloud, {10 /* max leaf */});

    int num_valid = 0;
    for (int j = 0; j < num_cloud_in; ++j) {
        const auto query_pcl = cloud_in.points[j];
        const num_t query_pt[3] = {query_pcl.x, query_pcl.y, query_pcl.z};

        size_t num_results = 1;
        std::vector<uint32_t> ret_index(num_results);
        std::vector<num_t> out_dist_sqr(num_results);

        num_results = index.knnSearch(
                &query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

        ret_index.resize(num_results);
        out_dist_sqr.resize(num_results);
        if (out_dist_sqr[0] < SQR_EPSILON) {  // identical point
            labels[j] = static_cast<uint32_t>(labeled_pc.points[ret_index[0]].intensity);
            ++num_valid;
        }
    }
    std::cout << "# of valid points: " << num_valid << std::endl;

    // KITTI-style 6-zero-padded filename
    const int NUM_ZEROS = 6;
    std::string count_str = std::to_string(frame_num);
    std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
    std::string abs_label_path = abs_dir + "/" + count_str_padded + ".label";

    // Pack instance labels into the upper 16 bits, leave semantic id zero.
    for (size_t i = 0; i < labels.size(); ++i) {
        labels[i] = labels[i] << 16;
    }

    std::cout << "\033[1;32m" << abs_label_path << "\033[0m" << std::endl;
    std::ofstream output_file(abs_label_path, std::ios::out | std::ios::binary);
    output_file.write(reinterpret_cast<char*>(&labels[0]), num_cloud_in * sizeof(uint32_t));
}

#endif  // TRAVEL_SAVE_LABELS_HPP
