//
// Created by shapelim on 6/23/21.
//
#include "utils/utils.hpp"

#ifndef TRAVEL_KITTI_LOADER_HPP
#define TRAVEL_KITTI_LOADER_HPP

class KittiLoader {
public:
    KittiLoader(const std::string &abs_path) {
        pc_path_ = abs_path + "/velodyne";
        label_path_ = abs_path + "/labels";

        for (num_frames_ = 0;; num_frames_++) {
            std::string filename = (boost::format("%s/%06d.bin") % pc_path_ % num_frames_).str();
            if (!boost::filesystem::exists(filename)) {
                break;
            }
        }
        int num_labels;
        for (num_labels = 0;; num_labels++) {
            std::string filename = (boost::format("%s/%06d.label") % label_path_ % num_labels).str();
            if (!boost::filesystem::exists(filename)) {
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

    ~KittiLoader() {}

    size_t size() const { return num_frames_; }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud(size_t i) const {
        std::string filename = (boost::format("%s/%06d.bin") % pc_path_ % i).str();
        FILE* file = fopen(filename.c_str(), "rb");
        if (!file) {
            std::cerr << "error: failed to load " << filename << std::endl;
            return nullptr;
        }

        std::vector<float> buffer(1000000); // Should be larger than 140,000 * 4
        size_t num_points = fread(reinterpret_cast<char*>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
        fclose(file);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        cloud_ptr->resize(num_points);

        for (int i = 0; i < num_points; i++) {
            auto& pt = cloud_ptr->at(i);
            pt.x = buffer[i * 4];
            pt.y = buffer[i * 4 + 1];
            pt.z = buffer[i * 4 + 2];
            pt.intensity = buffer[i * 4 + 3];
        }

        return cloud_ptr;
    }
//    template<typename T>
//    int loadPointCloud(size_t idx, pcl::PointCloud<T> &cloud) const {
//        std::string filename = (boost::format("%s/%06d.bin") % pc_path_ % idx).str();
//        FILE *file = fopen(filename.c_str(), "rb");
//        if (!file) {
//            std::cerr << "error: failed to load " << filename << std::endl;
//            return -1;
//        }
//        std::cout << filename.c_str() << std::endl;
//        std::vector<float> buffer(1000000);
//        size_t num_points = fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
//        fclose(file);
//        cloud.resize(num_points);
//        if (std::is_same<T, pcl::PointXYZ>::value) {
//            for (int i = 0; i < num_points; i++) {
//                auto &pt = cloud.at(i);
//                pt.x = buffer[i * 4];
//                pt.y = buffer[i * 4 + 1];
//                pt.z = buffer[i * 4 + 2];
//            }
//        } else if (std::is_same<T, pcl::PointXYZI>::value) {
//            for (int i = 0; i < num_points; i++) {
//                auto &pt = cloud.at(i);
//                pt.x = buffer[i * 4];
//                pt.y = buffer[i * 4 + 1];
//                pt.z = buffer[i * 4 + 2];
//                pt.intensity = buffer[i * 4 + 3];
//            }
//        } else if (std::is_same<T, PointXYZILID>::value) {
//            std::cout << "HEY????" << std::endl;
//            std::string label_name = (boost::format("%s/%06d.label") % label_path_ % idx).str();
//            std::cout << label_name << std::endl;
//            std::ifstream label_input(label_name, std::ios::binary);
//            if (!label_input.is_open()) {
//                std::cerr << "Labels are not available" << std::endl;
//                return -1;
//            }
//            label_input.seekg(0, std::ios::beg);
//
//            std::vector<uint32_t> labels(num_points);
//            label_input.read((char*)&labels[0], num_points * sizeof(uint32_t));
//
//            for (int i = 0; i < num_points; i++) {
//                auto &pt = cloud.at(i);
//                pt.x = buffer[i * 4];
//                pt.y = buffer[i * 4 + 1];
//                pt.z = buffer[i * 4 + 2];
//                pt.intensity = buffer[i * 4 + 3];
//                pt.label = labels[i] & 0xFFFF;
//                pt.id = labels[i] >> 16;
//            }
//        }
//        std::cout << "End!" << std::endl;
//        // std::cout << "Complete: " << cloud.points.size() << std::endl;
//    }

private:
    int num_frames_;
    std::string label_path_;
    std::string pc_path_;
};

#endif //TRAVEL_KITTI_LOADER_HPP
