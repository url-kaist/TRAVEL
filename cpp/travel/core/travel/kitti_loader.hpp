#ifndef TRAVEL_KITTI_LOADER_HPP
#define TRAVEL_KITTI_LOADER_HPP

// PCL-free, Boost-free KITTI velodyne loader. Uses C++17 <filesystem> and
// snprintf instead of boost::filesystem / boost::format so the core stays
// dependency-light enough to ship in a Windows wheel without vcpkg.

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "travel/types.hpp"

namespace {
inline std::string travel_kitti_format_path(const std::string& dir,
                                            const std::string& ext,
                                            int idx) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "/%06d.%s", idx, ext.c_str());
    return dir + buf;
}
}  // namespace

class KittiLoader {
public:
    explicit KittiLoader(const std::string& abs_path) {
        pc_path_    = abs_path + "/velodyne";
        label_path_ = abs_path + "/labels";

        for (num_frames_ = 0;; num_frames_++) {
            const auto filename = travel_kitti_format_path(pc_path_, "bin", num_frames_);
            if (!std::filesystem::exists(filename)) {
                break;
            }
        }
        int num_labels;
        for (num_labels = 0;; num_labels++) {
            const auto filename = travel_kitti_format_path(label_path_, "label", num_labels);
            if (!std::filesystem::exists(filename)) {
                break;
            }
        }

        if (num_frames_ == 0) {
            std::cerr << "\033[1;31mError: No files in " << pc_path_ << "\033[0m" << std::endl;
        }
        if (num_frames_ != num_labels) {
            std::cerr << "\033[1;31mError: The # of point clouds and # of labels are not same\033[0m" << std::endl;
        }
    }

    ~KittiLoader() = default;

    size_t size() const { return num_frames_; }

    travel::PointCloud<travel::PointXYZI>::ConstPtr cloud(size_t i) const {
        const auto filename = travel_kitti_format_path(pc_path_, "bin", static_cast<int>(i));
        FILE* file = std::fopen(filename.c_str(), "rb");
        if (!file) {
            std::cerr << "error: failed to load " << filename << std::endl;
            return nullptr;
        }

        std::vector<float> buffer(1000000);  // > 140k * 4 floats
        size_t num_points = std::fread(reinterpret_cast<char*>(buffer.data()),
                                       sizeof(float), buffer.size(), file) / 4;
        std::fclose(file);

        auto cloud_ptr = std::make_shared<travel::PointCloud<travel::PointXYZI>>();
        cloud_ptr->resize(num_points);

        for (size_t k = 0; k < num_points; k++) {
            auto& pt   = cloud_ptr->at(k);
            pt.x       = buffer[k * 4];
            pt.y       = buffer[k * 4 + 1];
            pt.z       = buffer[k * 4 + 2];
            pt.intensity = buffer[k * 4 + 3];
        }

        return cloud_ptr;
    }

private:
    int         num_frames_ = 0;
    std::string label_path_;
    std::string pc_path_;
};

#endif  // TRAVEL_KITTI_LOADER_HPP
