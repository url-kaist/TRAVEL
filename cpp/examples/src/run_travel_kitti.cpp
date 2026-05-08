// Standalone KITTI demo for the TRAVEL ground / above-ground object
// segmentation library. No ROS required.
//
// Usage:
//   run_travel_kitti <kitti_seq_dir> [frame_index] [output_dir]
//
//   <kitti_seq_dir>  directory containing velodyne/<XXXXXX>.bin files
//                    (the labels/ subdir is optional; not used here)
//   [frame_index]    optional, default 0
//   [output_dir]     optional, if given will write
//                       <output_dir>/ground_<idx>.pcd
//                       <output_dir>/nonground_<idx>.pcd
//                       <output_dir>/labeled_<idx>.pcd
//
// The aim of this example is twofold: (1) prove the C++ core builds and runs
// without any ROS dependency, and (2) give downstream users a concrete entry
// point for benchmarking on KITTI.

#define PCL_NO_PRECOMPILE

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "travel/aos.hpp"
#include "travel/kitti_loader.hpp"
#include "travel/point_types.hpp"
#include "travel/tgs.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <kitti_seq_dir> [frame_index] [output_dir]" << std::endl;
        return 1;
    }

    const std::string kitti_dir   = argv[1];
    const size_t      frame_idx   = (argc > 2) ? std::stoul(argv[2]) : 0;
    const std::string output_dir  = (argc > 3) ? argv[3] : "";

    KittiLoader loader(kitti_dir);
    if (loader.size() == 0) {
        std::cerr << "No velodyne frames found in " << kitti_dir << std::endl;
        return 2;
    }
    if (frame_idx >= loader.size()) {
        std::cerr << "frame_index " << frame_idx << " out of range (size = "
                  << loader.size() << ")" << std::endl;
        return 3;
    }

    auto xyzi = loader.cloud(frame_idx);
    if (!xyzi) {
        std::cerr << "Failed to load frame " << frame_idx << std::endl;
        return 4;
    }

    // Convert PointXYZI -> PointXYZILID (TRAVEL's working point type).
    pcl::PointCloud<PointXYZILID>::Ptr cloud_in(new pcl::PointCloud<PointXYZILID>());
    cloud_in->reserve(xyzi->size());
    for (const auto& p : xyzi->points) {
        PointXYZILID q;
        q.x         = p.x;
        q.y         = p.y;
        q.z         = p.z;
        q.intensity = p.intensity;
        q.label     = 0;
        q.id        = 0;
        cloud_in->emplace_back(q);
    }

    // ----- Ground segmentation (TGS) ---------------------------------------
    travel::TravelGroundSeg<PointXYZILID> tgs;
    // Defaults roughly mirror config/kitti_params.yaml, but tweak as needed.
    const double max_range = 80.0;
    const double min_range = 1.0;
    tgs.setParams(/*max_range=*/max_range,
                  /*min_range=*/min_range,
                  /*resolution=*/8.0,
                  /*num_iter=*/3,
                  /*num_lpr=*/5,
                  /*num_min_pts=*/10,
                  /*th_seeds=*/0.5,
                  /*th_dist=*/0.125,
                  /*th_outlier=*/0.3,
                  /*th_normal=*/0.940,
                  /*th_weight=*/200.0,
                  /*th_lcc_normal=*/0.03,
                  /*th_lcc_planar=*/0.1,
                  /*th_obstacle=*/1.0,
                  /*refine_mode=*/true,
                  /*viz_mode=*/false);

    pcl::PointCloud<PointXYZILID> ground;
    pcl::PointCloud<PointXYZILID> nonground;
    double tgs_time = 0.0;
    tgs.estimateGround(*cloud_in, ground, nonground, tgs_time);
    std::cout << "[TGS] frame " << frame_idx
              << " in=" << cloud_in->size()
              << " ground=" << ground.size()
              << " nonground=" << nonground.size()
              << " time=" << tgs_time << "s" << std::endl;

    // ----- Above-ground object segmentation (AOS) --------------------------
    travel::ObjectCluster<PointXYZILID> aos;
    aos.setParams(/*vert_scan=*/64,
                  /*horz_scan=*/4500,
                  /*min_range=*/static_cast<float>(min_range),
                  /*max_range=*/static_cast<float>(max_range),
                  /*min_vert_angle=*/-24.8f,
                  /*max_vert_angle=*/2.0f,
                  /*horz_merge_thres=*/0.4f,
                  /*vert_merge_thres=*/0.5f,
                  /*vert_scan_size=*/3,
                  /*horz_scan_size=*/5,
                  /*horz_extension_size=*/5,
                  /*horz_skip_size=*/5,
                  /*downsample=*/1,
                  /*min_cluster_size=*/10,
                  /*max_cluster_size=*/30000);

    pcl::PointCloud<PointXYZILID>::Ptr nonground_ptr(new pcl::PointCloud<PointXYZILID>(nonground));
    pcl::PointCloud<PointXYZILID>::Ptr labeled_ptr(new pcl::PointCloud<PointXYZILID>());
    aos.segmentObjects(nonground_ptr, labeled_ptr);
    std::cout << "[AOS] labeled points=" << labeled_ptr->size() << std::endl;

    // ----- Optional save ---------------------------------------------------
    if (!output_dir.empty()) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_xyzi(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr nonground_xyzi(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr labeled_xyzi(new pcl::PointCloud<pcl::PointXYZI>());
        PointXYZILID2XYZI(ground, ground_xyzi);
        PointXYZILID2XYZI(nonground, nonground_xyzi);
        PointXYZILID2XYZI(*labeled_ptr, labeled_xyzi);

        const std::string base = output_dir + "/" + std::to_string(frame_idx);
        pcl::io::savePCDFileBinary(base + "_ground.pcd",    *ground_xyzi);
        pcl::io::savePCDFileBinary(base + "_nonground.pcd", *nonground_xyzi);
        pcl::io::savePCDFileBinary(base + "_labeled.pcd",   *labeled_xyzi);
        std::cout << "Wrote PCDs under " << output_dir << std::endl;
    }

    return 0;
}
