#define PCL_NO_PRECOMPILE

#include "travel/aos.hpp"
#include "travel/tgs.hpp"
#include "travel/node.h"
#include "utils/utils.hpp"


ros::Publisher pub_nonground_cloud;
ros::Publisher pub_ground_cloud;
ros::Publisher pub_labeled_cloud;
ros::Publisher pub_raw_cloud;

boost::shared_ptr<travel::TravelGroundSeg<PointT>> travel_ground_seg;
boost::shared_ptr<travel::ObjectCluster<PointT>> travel_object_seg;

pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_in;
pcl::PointCloud<PointT>::Ptr ground_pc;
pcl::PointCloud<PointT>::Ptr nonground_pc;
pcl::PointCloud<PointT>::Ptr outlier_pc;
pcl::PointCloud<PointT>::Ptr labeled_pc;

float  min_range_, max_range_;
string abs_save_dir_;
bool   save_labels_ = false;

// Clusters findClusters(const MeshSegmenter::Config& config,
//                       const kimera_pgmo::MeshDelta& delta,
//                       const std::vector<size_t>& indices) {
//   pcl::IndicesPtr pcl_indices(new pcl::Indices(indices.begin(), indices.end()));

//   KdTreeT::Ptr tree(new KdTreeT());
//   tree->setInputCloud(delta.vertex_updates, pcl_indices);

//   pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> estimator;
//   estimator.setClusterTolerance(config.cluster_tolerance);
//   estimator.setMinClusterSize(config.min_cluster_size);
//   estimator.setMaxClusterSize(config.max_cluster_size);
//   estimator.setSearchMethod(tree);
//   estimator.setInputCloud(delta.vertex_updates);
//   estimator.setIndices(pcl_indices);

//   std::vector<pcl::PointIndices> cluster_indices;
//   estimator.extract(cluster_indices);

//   Clusters clusters;
//   clusters.resize(cluster_indices.size());
//   for (size_t k = 0; k < clusters.size(); ++k) {
//     auto& cluster = clusters.at(k);
//     const auto& curr_indices = cluster_indices.at(k).indices;
//     for (const auto local_idx : curr_indices) {
//       cluster.indices.push_back(delta.getGlobalIndex(local_idx));

//       const auto& p = delta.vertex_updates->at(local_idx);
//       const Eigen::Vector3d pos(p.x, p.y, p.z);
//       cluster.centroid += pos;
//     }

//     if (curr_indices.size()) {
//       cluster.centroid /= curr_indices.size();
//     }
//   }

//   return clusters;
// }

void callbackCloud(const sensor_msgs::PointCloud2ConstPtr &msg) {
    cloud_in->clear();
    ground_pc->clear();
    nonground_pc->clear();
    labeled_pc->clear();

    std_msgs::Header cloud_header = msg->header;

    // Convert to PCL
    pcl::fromROSMsg(*msg, *cloud_in);
    std::cout<< "Cloud size: " << cloud_in->size() << std::endl;
    pub_raw_cloud.publish(msg);

    PointT pt;
    for (auto &point : cloud_in->points) {
        bool is_nan = std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
        double pt_range = 0.0;
        if (is_nan){
            continue;
        }    
        pt_range = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
        if (pt_range <= min_range_ || pt_range >= max_range_){
            continue;
        }

        pt.x = point.x;
        pt.y = point.y;
        pt.z = point.z;
        // Non-ground points
        if (point.label == 0) {
          nonground_pc->emplace_back(pt);
        // Ground points
        } else {
          ground_pc->emplace_back(pt);
        }
    }

    sensor_msgs::PointCloud2 ground_cloud_msg;
    pcl::toROSMsg(*ground_pc, ground_cloud_msg);
    ground_cloud_msg.header = cloud_header;
    pub_ground_cloud.publish(ground_cloud_msg);

    sensor_msgs::PointCloud2 nonground_cloud_msg;
    pcl::toROSMsg(*nonground_pc, nonground_cloud_msg);
    nonground_cloud_msg.header = cloud_header;
    pub_nonground_cloud.publish(nonground_cloud_msg);

    // Apply above-ground object segmentation
    travel_object_seg->segmentObjects(nonground_pc, labeled_pc);
    std::cout << "\033[1;35m Above-Ground Seg: -> " << labeled_pc->size() << "\033[0m" << std::endl;

    sensor_msgs::PointCloud2 labeled_cloud_msg;
    pcl::toROSMsg(*labeled_pc, labeled_cloud_msg);
    labeled_cloud_msg.header = cloud_header;
    pub_labeled_cloud.publish(labeled_cloud_msg);

    return;
}

void callbackSegmentedCloud(const sensor_msgs::PointCloud2ConstPtr &msg) {

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmented_pc;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_pc;
    std_msgs::Header cloud_header = msg->header;

    // Convert to PCL
    pcl::fromROSMsg(*msg, *segmented_pc);
    
    PointT pt;
    static std::vector<uint32_t> object_labels ={2, 11, 14, 15, 16, 17, 18, 24, 26, 27, 29, 32, 33, 
                                                34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47};

    filtered_pc->reserve(segmented_pc->size());

    std::set<uint32_t> seen_labels;
    for (auto &point : segmented_pc->points) {
        bool is_nan = std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
        double pt_range = 0.0;
        if (is_nan){
            continue;
        }    
        pt_range = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
        if (pt_range <= min_range_ || pt_range >= 40.0){
            continue;
        }

        if (std::find(object_labeles.begin(), object_labeles.end(), point.label) != object_labeles.end()) {
           filtered_pc->emplace_back(point);
        }
    }

    auto getLabelIndices[&](const std::set<uint32_t>& desired_labels,
                            const kimera_pgmo::MeshDelta& delta,
                            const std::vector<size_t>& indices) {
        const auto& labels = delta.semantic_updates;

        std::map<uint32_t, std::vector<size_t>> label_indices;
        std::set<uint32_t> seen_labels;
        for (const auto idx : indices) {
          if (static_cast<size_t>(idx) >= labels.size()) {
            LOG(ERROR) << "bad index " << idx << " (of " << labels.size() << ")";
            continue;
          }

          const auto label = labels[idx];
          seen_labels.insert(label);
          if (!desired_labels.count(label)) {
            continue;
          }

          auto iter = label_indices.find(label);
          if (iter == label_indices.end()) {
            iter = label_indices.emplace(label, std::vector<size_t>()).first;
          }

          iter->second.push_back(idx);
        }

        VLOG(2) << "[Mesh Segmenter] Seen labels: " << printLabels(seen_labels);
        return label_indices;
    };

    getLabelIndices(desired);

     
    for (const auto label : labels_) {
        if (!label_indices.count(label)) {
          continue;
    }

    if (label_indices.at(label).size() < config.min_cluster_size) {
        continue;
    }

    const auto clusters = findClusters(config, delta, label_indices.at(label));

    VLOG(2) << "[Mesh Segmenter]  - Found " << clusters.size()
            << " cluster(s) of label " << static_cast<int>(label);
    label_clusters.insert({label, clusters});
  }

    travel_object_seg->segmentObjects(nonground_pc, labeled_pc);
    std::cout << "\033[1;35m Above-Ground Seg: -> " << labeled_pc->size() << "\033[0m" << std::endl;

    sensor_msgs::PointCloud2 labeled_cloud_msg;
    pcl::toROSMsg(*labeled_pc, labeled_cloud_msg);
    labeled_cloud_msg.header = cloud_header;
    pub_labeled_cloud.publish(labeled_cloud_msg);

    return;
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "travel_graph_clsuter");
    ros::NodeHandle nh;
    ROS_INFO("Travel: Graph Cluster");

    // Set Parameters
    std::string cloud_topic_;
    std::string labeled_cloud_topic_;
    nh.param<string> ("/node_topic"  , cloud_topic_, "/ouster/points");
    nh.param<string> ("/labeled_cloud_topic"  , labeled_cloud_topic_, "");
labeled_cloud
    std::cout << "\033[1;32m" << "Cloud topic: " << cloud_topic_ << "\033[0m" << std::endl;
    nh.param<bool> ("/save_results/save_labels"  , save_labels_, false);
    nh.param<string> ("/save_results/abs_save_dir"  , abs_save_dir_, "");

    int vert_scan, horz_scan;
    float min_vert_angle, max_vert_angle;
    nh.param<float> ("/lidar/min_range"     , min_range_, 0.0);
    nh.param<float> ("/lidar/max_range"     , max_range_, 30.0);
    nh.param<int>   ("/lidar/vert_scan"     , vert_scan, 64);
    nh.param<int>   ("/lidar/horz_scan"     , horz_scan, 1800);
    nh.param<float> ("/lidar/min_vert_angle", min_vert_angle, -30.0);
    nh.param<float> ("/lidar/max_vert_angle", max_vert_angle, 50.0);

    float tgf_res, th_seeds, th_dist, th_outlier, th_normal, th_weight, th_lcc_normal_similarity, th_lcc_planar_dist, th_obstacle;
    int num_iter, num_lpr, num_min_pts;
    bool refine_mode, viz_mode;
    nh.param<float>("/tgs/resolution"    , tgf_res, 5.0);
    nh.param<int>  ("/tgs/num_iter"     , num_iter, 3);
    nh.param<int>  ("/tgs/num_lpr"      , num_lpr, 20);
    nh.param<int>  ("/tgs/num_min_pts"  , num_min_pts, 10);
    nh.param<float>("/tgs/th_seeds"     , th_seeds, 0.4);
    nh.param<float>("/tgs/th_dist"      , th_dist, 0.3);
    nh.param<float>("/tgs/th_outlier"   , th_outlier, 0.3);
    nh.param<float>("/tgs/th_normal"    , th_normal, 0.707);
    nh.param<float>("/tgs/th_weight"    , th_weight, 1.5);
    nh.param<float>("/tgs/th_obstacle"  , th_obstacle, 1.5);
    nh.param<float>("/tgs/th_lcc_normal", th_lcc_normal_similarity , 1.5);
    nh.param<float>("/tgs/th_lcc_planar", th_lcc_planar_dist , 1.5);
    nh.param<bool> ("/tgs/refine_mode"  , refine_mode, true);
    nh.param<bool> ("/tgs/visualization", viz_mode, true);  

    float car_width, car_length, lidar_width_offset, lidar_length_offset, horz_merge_thres, vert_merge_thres;
    int downsample, vert_scan_size, horz_scan_size, horz_skip_size, horz_extension_size, min_cluster_size, max_cluster_size;
    bool debug;
    nh.param<int>  ("/aos/downsample"           , downsample, 2);
    nh.param<float>("/aos/car_width"            , car_width, 1.0);
    nh.param<float>("/aos/car_length"           , car_length, 1.0);
    nh.param<float>("/aos/lidar_width_offset"   , lidar_width_offset, 0.0);
    nh.param<float>("/aos/lidar_length_offset"  , lidar_length_offset, 0.0);
    nh.param<float>("/aos/th_horz_merg"         , horz_merge_thres, 0.3);
    nh.param<float>("/aos/th_vert_merg"         , vert_merge_thres, 1.0);
    nh.param<int>  ("/aos/vert_scan_size"       , vert_scan_size, 4);
    nh.param<int>  ("/aos/horz_scan_size"       , horz_scan_size, 4);
    nh.param<int>  ("/aos/horz_skip_size"       , horz_skip_size, 4);
    nh.param<int>  ("/aos/horz_extension_size"  , horz_extension_size, 3);
    nh.param<int>  ("/aos/min_cluster_size"     , min_cluster_size, 4);
    nh.param<int>  ("/aos/max_cluster_size"     , max_cluster_size, 100);

    travel_ground_seg.reset(new travel::TravelGroundSeg<PointT>());
    // travel_ground_seg.reset(new travel::TravelGroundSeg<PointT>(&nh));
    travel_object_seg.reset(new travel::ObjectCluster<PointT>());

    std::cout << "Max Range: " << max_range_ << std::endl;
    std::cout << "Min Range: " << min_range_ << std::endl;
    travel_ground_seg->setParams(max_range_, min_range_, tgf_res, 
                                num_iter, num_lpr, num_min_pts, th_seeds, 
                                th_dist, th_outlier, th_normal, th_weight, 
                                th_lcc_normal_similarity, th_lcc_planar_dist, th_obstacle,
                                refine_mode, viz_mode);

    travel_object_seg->setParams(vert_scan, horz_scan, min_range_, max_range_, 
                                min_vert_angle, max_vert_angle,
                                horz_merge_thres, vert_merge_thres, vert_scan_size,
                                horz_scan_size, horz_extension_size, horz_skip_size, downsample, 
                                min_cluster_size, max_cluster_size);
    
    cloud_in.reset(new pcl::PointCloud<pcl::PointXYZL>());
    ground_pc.reset(new pcl::PointCloud<PointT>());
    nonground_pc.reset(new pcl::PointCloud<PointT>());
    labeled_pc.reset(new pcl::PointCloud<PointT>());

    pub_raw_cloud = nh.advertise<sensor_msgs::PointCloud2>("travel/original_pc", 1);
    pub_nonground_cloud = nh.advertise<sensor_msgs::PointCloud2>("travel/nonground_pc", 1);
    pub_ground_cloud = nh.advertise<sensor_msgs::PointCloud2>("travel/ground_pc", 1);
    pub_labeled_cloud = nh.advertise<sensor_msgs::PointCloud2>("travel/segmented_pc", 1);

    ros::Subscriber GrundLabeledCloudSub = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic_, 4000, callbackCloud, ros::TransportHints().tcpNoDelay());

    ros::Subscriber SegmentedCloudSub = nh.subscribe<sensor_msgs::PointCloud2>(labeled_cloud_topic_, 4000, callbackSegmentedCloud, ros::TransportHints().tcpNoDelay());
    ros::spin();

    return 0;
}
