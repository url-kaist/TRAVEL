// ROS 2 wrapper around travel::TravelGroundSeg + travel::ObjectCluster.
//
// Topic flow:
//   sub:  ~/input  (sensor_msgs/PointCloud2)        -- arbitrary lidar frame
//   pub:  ~/ground       (sensor_msgs/PointCloud2)  -- ground-classified pts
//   pub:  ~/nonground    (sensor_msgs/PointCloud2)  -- everything else
//   pub:  ~/labeled      (sensor_msgs/PointCloud2)  -- per-cluster labeled
//                                                     (ID stored in the
//                                                     PointXYZILID 'id' field)
//
// All algorithm parameters are exposed as ROS 2 node parameters with the
// same defaults as ros/config/kitti_params.yaml. AOS::setSeed() is invoked
// when the `aos_seed` parameter is set to a non-negative value.

#define PCL_NO_PRECOMPILE

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "travel/aos.hpp"
#include "travel/point_types.hpp"
#include "travel/tgs.hpp"

using PointT = PointXYZILID;
using std::placeholders::_1;

class TravelNode : public rclcpp::Node {
public:
    TravelNode() : rclcpp::Node("travel_node") {
        // -- Generic ranges
        const auto max_range = declare_parameter("lidar.max_range", 80.0);
        const auto min_range = declare_parameter("lidar.min_range", 1.0);
        const auto vert_scan = declare_parameter("lidar.vert_scan", 64);
        const auto horz_scan = declare_parameter("lidar.horz_scan", 4500);
        const auto min_vert_angle = declare_parameter("lidar.min_vert_angle", -24.8);
        const auto max_vert_angle = declare_parameter("lidar.max_vert_angle", 2.0);

        // -- TGS parameters
        const auto tgf_res     = declare_parameter("tgs.resolution",   8.0);
        const auto num_iter    = declare_parameter("tgs.num_iter",     3);
        const auto num_lpr     = declare_parameter("tgs.num_lpr",      5);
        const auto num_min_pts = declare_parameter("tgs.num_min_pts",  10);
        const auto th_seeds    = declare_parameter("tgs.th_seeds",     0.5);
        const auto th_dist     = declare_parameter("tgs.th_dist",      0.125);
        const auto th_outlier  = declare_parameter("tgs.th_outlier",   0.3);
        const auto th_normal   = declare_parameter("tgs.th_normal",    0.940);
        const auto th_weight   = declare_parameter("tgs.th_weight",    200.0);
        const auto th_lcc_n    = declare_parameter("tgs.th_lcc_normal", 0.03);
        const auto th_lcc_p    = declare_parameter("tgs.th_lcc_planar", 0.1);
        const auto th_obstacle = declare_parameter("tgs.th_obstacle",   1.0);
        const auto refine_mode = declare_parameter("tgs.refine_mode",   true);

        // -- AOS parameters
        const auto horz_merge_thres   = declare_parameter("aos.th_horz_merg",     0.4);
        const auto vert_merge_thres   = declare_parameter("aos.th_vert_merg",     0.5);
        const auto vert_scan_size     = declare_parameter("aos.vert_scan_size",   3);
        const auto horz_scan_size     = declare_parameter("aos.horz_scan_size",   5);
        const auto horz_extension_sz  = declare_parameter("aos.horz_extension_size", 5);
        const auto horz_skip_size     = declare_parameter("aos.horz_skip_size",   5);
        const auto downsample         = declare_parameter("aos.downsample",       1);
        const auto min_cluster_size   = declare_parameter("aos.min_cluster_size", 10);
        const auto max_cluster_size   = declare_parameter("aos.max_cluster_size", 30000);
        // -1 keeps the default std::random_device shuffle. Any non-negative
        // int pins the AOS cluster-id shuffle so consecutive runs produce
        // bit-identical labeled clouds — useful for testing and recording.
        const auto aos_seed = declare_parameter<int>("aos.seed", -1);

        tgs_.setParams(max_range, min_range, tgf_res,
                       static_cast<int>(num_iter),
                       static_cast<int>(num_lpr),
                       static_cast<int>(num_min_pts),
                       th_seeds, th_dist, th_outlier,
                       th_normal, th_weight,
                       th_lcc_n, th_lcc_p, th_obstacle,
                       refine_mode, /*viz_mode=*/false);

        aos_.setParams(static_cast<int>(vert_scan),
                       static_cast<int>(horz_scan),
                       static_cast<float>(min_range),
                       static_cast<float>(max_range),
                       static_cast<float>(min_vert_angle),
                       static_cast<float>(max_vert_angle),
                       static_cast<float>(horz_merge_thres),
                       static_cast<float>(vert_merge_thres),
                       static_cast<int>(vert_scan_size),
                       static_cast<int>(horz_scan_size),
                       static_cast<int>(horz_extension_sz),
                       static_cast<int>(horz_skip_size),
                       static_cast<int>(downsample),
                       static_cast<int>(min_cluster_size),
                       static_cast<int>(max_cluster_size));

        if (aos_seed >= 0) {
            aos_.setSeed(static_cast<uint32_t>(aos_seed));
            RCLCPP_INFO(get_logger(), "AOS shuffle seed pinned to %d", aos_seed);
        }

        min_range_ = static_cast<float>(min_range);
        max_range_ = static_cast<float>(max_range);

        rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "~/input", sensor_qos,
            std::bind(&TravelNode::cloudCallback, this, _1));

        pub_ground_    = create_publisher<sensor_msgs::msg::PointCloud2>("~/ground",    sensor_qos);
        pub_nonground_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/nonground", sensor_qos);
        pub_labeled_   = create_publisher<sensor_msgs::msg::PointCloud2>("~/labeled",   sensor_qos);

        RCLCPP_INFO(get_logger(),
                    "travel_node ready (max_range=%.1f, min_range=%.1f)",
                    max_range, min_range);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        // The travel core stopped using PCL types in v1.1, so this wrapper
        // does the PCL <-> travel conversion at the ROS boundary. PCL stays
        // in this file (we still need pcl::fromROSMsg / pcl::toROSMsg),
        // it just doesn't leak into the algorithm any more.

        // 1. ROS message -> pcl::PointXYZ buffer for cheap deserialization,
        //    then funnel into a travel::PointCloud<PointXYZILID> with the
        //    range / NaN filter applied on the fly.
        pcl::PointCloud<pcl::PointXYZ> raw;
        pcl::fromROSMsg(*msg, raw);

        auto cloud_in = std::make_shared<travel::PointCloud<PointT>>();
        cloud_in->reserve(raw.size());
        for (const auto& p : raw.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            const float r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (r < min_range_ || r > max_range_) continue;
            PointT q{};
            q.x = p.x; q.y = p.y; q.z = p.z;
            q.intensity = 0.0f; q.label = 0; q.id = 0;
            cloud_in->push_back(q);
        }

        // 2. Ground segmentation
        travel::PointCloud<PointT> ground, nonground;
        double tgs_time = 0.0;
        const auto t0 = std::chrono::steady_clock::now();
        tgs_.estimateGround(*cloud_in, ground, nonground, tgs_time);

        // 3. Object clustering on the non-ground subset
        auto nonground_ptr = std::make_shared<travel::PointCloud<PointT>>(nonground);
        auto labeled_ptr   = std::make_shared<travel::PointCloud<PointT>>();
        aos_.segmentObjects(nonground_ptr, labeled_ptr);
        const auto t1 = std::chrono::steady_clock::now();
        const double total_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;

        // 4. Publish all three streams. Headers carry the input timestamp /
        //    frame_id so tooling stays in sync.
        publishCloud(*pub_ground_,    ground,        msg->header, /*emit_label=*/false);
        publishCloud(*pub_nonground_, nonground,     msg->header, /*emit_label=*/false);
        publishCloud(*pub_labeled_,   *labeled_ptr,  msg->header, /*emit_label=*/true);

        RCLCPP_DEBUG(get_logger(),
                     "in=%zu ground=%zu nonground=%zu labeled=%zu  total=%.2fms",
                     cloud_in->size(), ground.size(), nonground.size(),
                     labeled_ptr->size(), total_ms);
    }

    // Converts a travel::PointCloud<PointXYZILID> to a sensor_msgs PointCloud2.
    // Uses PCL types under the hood for the field-layout machinery that
    // pcl::toROSMsg expects:
    //   * ground / nonground -> pcl::PointXYZI (intensity carries the
    //     algorithm-input intensity, currently 0).
    //   * labeled            -> pcl::PointXYZL (label carries the cluster id).
    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& pub,
                      const travel::PointCloud<PointT>& cloud,
                      const std_msgs::msg::Header& header,
                      bool emit_label) {
        if (pub.get_subscription_count() == 0) return;
        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        if (emit_label) {
            pcl::PointCloud<pcl::PointXYZL> out;
            out.points.reserve(cloud.size());
            for (const auto& p : cloud.points) {
                pcl::PointXYZL q;
                q.x = p.x; q.y = p.y; q.z = p.z;
                q.label = p.id;
                out.points.push_back(q);
            }
            out.width = static_cast<std::uint32_t>(out.points.size());
            out.height = 1;
            pcl::toROSMsg(out, *msg);
        } else {
            pcl::PointCloud<pcl::PointXYZI> out;
            out.points.reserve(cloud.size());
            for (const auto& p : cloud.points) {
                pcl::PointXYZI q;
                q.x = p.x; q.y = p.y; q.z = p.z;
                q.intensity = p.intensity;
                out.points.push_back(q);
            }
            out.width = static_cast<std::uint32_t>(out.points.size());
            out.height = 1;
            pcl::toROSMsg(out, *msg);
        }
        msg->header = header;
        pub.publish(std::move(msg));
    }

    travel::TravelGroundSeg<PointT> tgs_;
    travel::ObjectCluster<PointT>   aos_;

    float min_range_ = 0.0f;
    float max_range_ = 0.0f;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_nonground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_labeled_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TravelNode>());
    rclcpp::shutdown();
    return 0;
}
