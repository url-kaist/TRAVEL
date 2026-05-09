// Standalone KITTI demo for the TRAVEL ground / above-ground object
// segmentation library. PCL-free; only depends on the travel core (Eigen).
//
// Usage:
//   run_travel_kitti <kitti_seq_dir> [frame_index] [output_dir]
//
//   <kitti_seq_dir>  directory containing velodyne/<XXXXXX>.bin files
//                    (the labels/ subdir is optional; not used here)
//   [frame_index]    optional, default 0
//   [output_dir]     optional, if given will write
//                       <output_dir>/<idx>_ground.bin     (Nx4 float32 XYZI)
//                       <output_dir>/<idx>_nonground.bin  (same)
//                       <output_dir>/<idx>_labeled.bin    (Nx5 float32 XYZI + cluster_id-as-float)
//
// The aim of this example is twofold: (1) prove the C++ core builds and runs
// without any ROS or PCL dependency, and (2) give downstream users a concrete
// entry point for benchmarking on KITTI.

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "travel/aos.hpp"
#include "travel/kitti_loader.hpp"
#include "travel/point_types.hpp"
#include "travel/tgs.hpp"

namespace {

// Write an Nx4 float32 buffer (XYZI). Same shape as KITTI velodyne .bin so
// downstream tooling can reuse the existing loaders.
void writeXYZI(const std::string& path, const travel::PointCloud<PointXYZILID>& cloud) {
    std::ofstream of(path, std::ios::binary);
    for (const auto& p : cloud.points) {
        const float row[4] = {p.x, p.y, p.z, p.intensity};
        of.write(reinterpret_cast<const char*>(row), sizeof(row));
    }
}

// Nx5 float32: XYZI + cluster_id as float for trivial inspection.
void writeXYZIID(const std::string& path, const travel::PointCloud<PointXYZILID>& cloud) {
    std::ofstream of(path, std::ios::binary);
    for (const auto& p : cloud.points) {
        const float row[5] = {p.x, p.y, p.z, p.intensity, static_cast<float>(p.id)};
        of.write(reinterpret_cast<const char*>(row), sizeof(row));
    }
}

}  // namespace

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
    auto cloud_in = std::make_shared<travel::PointCloud<PointXYZILID>>();
    cloud_in->reserve(xyzi->size());
    for (const auto& p : xyzi->points) {
        PointXYZILID q{};
        q.x = p.x; q.y = p.y; q.z = p.z;
        q.intensity = p.intensity;
        q.label = 0;
        q.id    = 0;
        cloud_in->emplace_back(q);
    }

    // ----- Ground segmentation (TGS) ---------------------------------------
    travel::TravelGroundSeg<PointXYZILID> tgs;
    const double max_range = 80.0;
    const double min_range = 1.0;
    tgs.setParams(max_range, min_range, /*resolution=*/8.0,
                  /*num_iter=*/3, /*num_lpr=*/5, /*num_min_pts=*/10,
                  /*th_seeds=*/0.5, /*th_dist=*/0.125, /*th_outlier=*/0.3,
                  /*th_normal=*/0.940, /*th_weight=*/200.0,
                  /*th_lcc_normal=*/0.03, /*th_lcc_planar=*/0.1, /*th_obstacle=*/1.0,
                  /*refine_mode=*/true, /*viz_mode=*/false);

    travel::PointCloud<PointXYZILID> ground;
    travel::PointCloud<PointXYZILID> nonground;
    double tgs_time = 0.0;
    tgs.estimateGround(*cloud_in, ground, nonground, tgs_time);
    std::cout << "[TGS] frame " << frame_idx
              << " in=" << cloud_in->size()
              << " ground=" << ground.size()
              << " nonground=" << nonground.size()
              << " time=" << tgs_time << "s" << std::endl;

    // ----- Above-ground object segmentation (AOS) --------------------------
    travel::ObjectCluster<PointXYZILID> aos;
    aos.setParams(/*vert_scan=*/64, /*horz_scan=*/4500,
                  static_cast<float>(min_range), static_cast<float>(max_range),
                  -24.8f, 2.0f, 0.4f, 0.5f, 3, 5, 5, 5, 1, 10, 30000);

    auto nonground_ptr = std::make_shared<travel::PointCloud<PointXYZILID>>(nonground);
    auto labeled_ptr   = std::make_shared<travel::PointCloud<PointXYZILID>>();
    aos.segmentObjects(nonground_ptr, labeled_ptr);
    std::cout << "[AOS] labeled points=" << labeled_ptr->size() << std::endl;

    // ----- Optional save ---------------------------------------------------
    if (!output_dir.empty()) {
        const std::string base = output_dir + "/" + std::to_string(frame_idx);
        writeXYZI(base + "_ground.bin",    ground);
        writeXYZI(base + "_nonground.bin", nonground);
        writeXYZIID(base + "_labeled.bin", *labeled_ptr);
        std::cout << "Wrote " << base << "_{ground,nonground,labeled}.bin" << std::endl;
    }

    return 0;
}
