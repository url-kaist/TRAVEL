#ifndef TRAVEL_GSEG_H
#define TRAVEL_GSEG_H
//
// Created by Minho Oh & Euigon Jung on 1/31/22.
// We really appreciate Hyungtae Lim and Prof. Hyun Myung! :)
//
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <mutex>
#include <thread>
#include <chrono>
#include <math.h>
#include <fstream>
#include <memory>
#include <signal.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <jsk_recognition_msgs/PolygonArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>

namespace travel {

    #define PTCLOUD_SIZE 132000
    #define NODEWISE_PTCLOUDSIZE 5000

    #define UNKNOWN 1
    #define NONGROUND 2
    #define GROUND 3

    using Eigen::MatrixXf;
    using Eigen::JacobiSVD;
    using Eigen::VectorXf;

    template <typename PointType>
    bool point_z_cmp(PointType a, PointType b){
        return a.z<b.z;
    }

    struct TriGridIdx {
        int row, col, tri;
    };

    struct TriGridEdge {
        std::pair<TriGridIdx, TriGridIdx> Pair;
        bool is_traversable;
    };

    template <typename PointType>
    struct TriGridNode {
        int node_type;
        pcl::PointCloud<PointType> ptCloud;

        bool is_curr_data;
        
        // planar model
        Eigen::Vector3f normal;
        Eigen::Vector3f mean_pt;
        double d;

        Eigen::Vector3f singular_values;
        Eigen::Matrix3f eigen_vectors;
        double weight;

        double th_dist_d;
        double th_outlier_d;

        // graph_searching
        bool need_recheck;
        bool is_visited;
        bool is_rejection;
        int check_life;
        int depth;
    };

    struct TriGridCorner {
        double x, y;
        std::vector<double> zs;
        std::vector<double> weights;
    };

    template <typename PointType>
    using GridNode = std::vector<TriGridNode<PointType>>;

    template <typename PointType>
    using TriGridField = std::vector<std::vector<GridNode<PointType>>>;

    template <typename PointType>
    class TravelGroundSeg{
    private:
        // ros::NodeHandle node_handle_;
        pcl::PCLHeader cloud_header_;
        std_msgs::Header msg_header_;

        // ros::Publisher pub_trigrid_nodes_;
        // ros::Publisher pub_trigrid_edges_;
        // ros::Publisher pub_trigrid_corners_;
        // ros::Publisher pub_tgseg_ground_cloud;
        // ros::Publisher pub_tgseg_nonground_cloud;
        // ros::Publisher pub_tgseg_outliers_cloud;
        
        jsk_recognition_msgs::PolygonArray viz_trigrid_polygons_;
        visualization_msgs::Marker viz_trigrid_edges_;
        pcl::PointCloud<pcl::PointXYZ> viz_trigrid_corners_;

        bool REFINE_MODE_;
        bool VIZ_MDOE_;
        
        double MAX_RANGE_;
        double MIN_RANGE_;
        double TGF_RESOLUTION_;

        int NUM_ITER_;
        int NUM_LRP_;
        int NUM_MIN_POINTS_;
        
        double TH_SEEDS_;
        double TH_DIST_;
        double TH_OUTLIER_;
        
        double TH_NORMAL_;
        double TH_WEIGHT_;
        double TH_LCC_NORMAL_SIMILARITY_;
        double TH_LCC_PLANAR_MODEL_DIST_;
        double TH_OBSTACLE_HEIGHT_;

        TriGridField<PointType> trigrid_field_;
        std::vector<TriGridEdge> trigrid_edges_;
        std::vector<std::vector<TriGridCorner>> trigrid_corners_;
        std::vector<std::vector<TriGridCorner>> trigrid_centers_;

        pcl::PointCloud<PointType> empty_cloud_;
        TriGridNode<PointType>  empty_trigrid_node_;
        GridNode<PointType> empty_grid_nodes_;
        TriGridCorner empty_trigrid_corner_;
        TriGridCorner empty_trigrid_center_;

        pcl::PointCloud<PointType> ptCloud_tgfwise_ground_;
        pcl::PointCloud<PointType> ptCloud_tgfwise_nonground_;
        pcl::PointCloud<PointType> ptCloud_tgfwise_outliers_;
        pcl::PointCloud<PointType> ptCloud_tgfwise_obstacle_;
        pcl::PointCloud<PointType> ptCloud_nodewise_ground_;
        pcl::PointCloud<PointType> ptCloud_nodewise_nonground_;
        pcl::PointCloud<PointType> ptCloud_nodewise_outliers_;
        pcl::PointCloud<PointType> ptCloud_nodewise_obstacle_;

    public:
        // TravelGroundSeg(ros::NodeHandle* nh):node_handle_(*nh){
        TravelGroundSeg(){
            // Init ROS related
            ROS_INFO("Inititalizing Traversable Ground Segmentation...");

            // pub_trigrid_nodes_  = node_handle_.advertise<jsk_recognition_msgs::PolygonArray>("/travelgseg/nodes", 1);
            // pub_trigrid_edges_  = node_handle_.advertise<visualization_msgs::Marker>("/travelgseg/edges", 1);
            // pub_trigrid_corners_= node_handle_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/travelgseg/corners", 1);
            
            // pub_tgseg_ground_cloud    = node_handle_.advertise<sensor_msgs::PointCloud2>("/travelgseg/ground_cloud", 1);
            // pub_tgseg_nonground_cloud = node_handle_.advertise<sensor_msgs::PointCloud2>("/travelgseg/nonground_cloud", 1);
            // pub_tgseg_outliers_cloud   = node_handle_.advertise<sensor_msgs::PointCloud2>("/travelgseg/outlier_cloud", 1);
        };

        void setParams(const double max_range, const double min_range, const double resolution, 
                            const int num_iter, const int num_lpr, const int num_min_pts, const double th_seeds, 
                            const double th_dist, const double th_outlier, const double th_normal, const double th_weight, 
                            const double th_lcc_normal_similiarity, const double th_lcc_planar_model_dist, const double th_obstacle,
                            const bool refine_mode, const bool visualization) {
            std::cout<<""<<std::endl;
            ROS_INFO("Set TRAVEL_GSEG Parameters");

            MAX_RANGE_ = max_range;
            ROS_INFO("Max Range: %f", MAX_RANGE_);

            MIN_RANGE_ = min_range;
            ROS_INFO("Min Range: %f", MIN_RANGE_);
            
            TGF_RESOLUTION_ = resolution;
            ROS_INFO("Resolution: %f", TGF_RESOLUTION_);
            
            NUM_ITER_ = num_iter;
            ROS_INFO("Num of Iteration: %d", NUM_ITER_);
            
            NUM_LRP_ = num_lpr;
            ROS_INFO("Num of LPR: %d", NUM_LRP_);
            
            NUM_MIN_POINTS_ = num_min_pts;
            ROS_INFO("Num of min. points: %d", NUM_MIN_POINTS_);
            
            TH_SEEDS_ = th_seeds;
            ROS_INFO("Seeds Threshold: %f", TH_SEEDS_);
            
            TH_DIST_ = th_dist;
            ROS_INFO("Distance Threshold: %f", TH_DIST_);
            
            TH_OUTLIER_ = th_outlier;
            ROS_INFO("Outlier Threshold: %f", TH_OUTLIER_);

            TH_NORMAL_ = th_normal;
            ROS_INFO("Normal Threshold: %f", TH_NORMAL_);

            TH_WEIGHT_ = th_weight;
            ROS_INFO("Node Weight Threshold: %f", TH_WEIGHT_);

            TH_LCC_NORMAL_SIMILARITY_ = th_lcc_normal_similiarity;
            ROS_INFO("LCC Normal Similarity: %f", TH_LCC_NORMAL_SIMILARITY_);

            TH_LCC_PLANAR_MODEL_DIST_ = th_lcc_planar_model_dist;
            ROS_INFO("LCC Plane Distance   : %f", TH_LCC_PLANAR_MODEL_DIST_);

            TH_OBSTACLE_HEIGHT_ = th_obstacle;
            ROS_INFO("Obstacle Max Height  : %f", TH_OBSTACLE_HEIGHT_);
            
            REFINE_MODE_ = refine_mode;
            VIZ_MDOE_ = visualization;

            ROS_INFO("Set TGF Parameters");
            initTriGridField(trigrid_field_);
            initTriGridCorners(trigrid_corners_, trigrid_centers_);

            ptCloud_tgfwise_ground_.clear();
            ptCloud_tgfwise_ground_.reserve(PTCLOUD_SIZE);
            ptCloud_tgfwise_nonground_.clear();
            ptCloud_tgfwise_nonground_.reserve(PTCLOUD_SIZE);
            ptCloud_tgfwise_outliers_.clear();
            ptCloud_tgfwise_outliers_.reserve(PTCLOUD_SIZE);
            ptCloud_tgfwise_obstacle_.clear();
            ptCloud_tgfwise_obstacle_.reserve(PTCLOUD_SIZE);
    
            ptCloud_nodewise_ground_.clear();
            ptCloud_nodewise_ground_.reserve(NODEWISE_PTCLOUDSIZE);
            ptCloud_nodewise_nonground_.clear();
            ptCloud_nodewise_nonground_.reserve(NODEWISE_PTCLOUDSIZE);
            ptCloud_nodewise_outliers_.clear();
            ptCloud_nodewise_outliers_.reserve(NODEWISE_PTCLOUDSIZE);
            ptCloud_nodewise_obstacle_.clear();
            ptCloud_nodewise_obstacle_.reserve(NODEWISE_PTCLOUDSIZE);

            if (VIZ_MDOE_) {
                viz_trigrid_polygons_.polygons.clear();
                viz_trigrid_polygons_.polygons.reserve(rows_ * cols_);
                viz_trigrid_polygons_.likelihood.clear();
                viz_trigrid_polygons_.likelihood.reserve(rows_ * cols_);
                

                viz_trigrid_edges_.ns = "trigrid_edges";
                viz_trigrid_edges_.action = visualization_msgs::Marker::ADD;
                viz_trigrid_edges_.type = visualization_msgs::Marker::LINE_LIST;
                viz_trigrid_edges_.pose.orientation.w = 1.0;
                viz_trigrid_edges_.scale.x = 0.5;
                viz_trigrid_edges_.id = 0;

                viz_trigrid_corners_.clear();
                viz_trigrid_corners_.reserve(rows_*cols_ + (rows_+ 1)*(cols_+1));
            }
        };

        void estimateGround(const pcl::PointCloud<PointType>& cloud_in,
                            pcl::PointCloud<PointType>& cloudGround_out,
                            pcl::PointCloud<PointType>& cloudNonground_out,
                            double& time_taken){
        
            // 0. Init
            static time_t start, end;
            cloud_header_ = cloud_in.header;
            pcl_conversions::fromPCL(cloud_header_, msg_header_);
            // ROS_INFO("TriGrid Field-based Traversable Ground Segmentation...");
            start = clock();
            ptCloud_tgfwise_outliers_.clear();
            ptCloud_tgfwise_outliers_.reserve(cloud_in.size());


            // 1. Embed PointCloud to TriGridField
            clearTriGridField(trigrid_field_);
            clearTriGridCorners(trigrid_corners_, trigrid_centers_);

            embedCloudToTriGridField(cloud_in, trigrid_field_);

            // 2. Node-wise Terrain Modeling
            modelNodeWiseTerrain(trigrid_field_);
            
            // 3. Breadth-first Traversable Graph Search
            BreadthFirstTraversableGraphSearch(trigrid_field_);
            setTGFCornersCenters(trigrid_field_, trigrid_corners_, trigrid_centers_);

            // 4. TGF-wise Traversable Terrain Model Fitting
            if (REFINE_MODE_){
                fitTGFWiseTraversableTerrainModel(trigrid_field_, trigrid_corners_, trigrid_centers_);
            }
            
            // 5. Ground Segmentation
            segmentTGFGround(trigrid_field_, ptCloud_tgfwise_ground_, ptCloud_tgfwise_nonground_, ptCloud_tgfwise_obstacle_, ptCloud_tgfwise_outliers_);
            cloudGround_out = ptCloud_tgfwise_ground_;
            cloudNonground_out = ptCloud_tgfwise_nonground_;
            cloudGround_out.header = cloudNonground_out.header = cloud_header_;

            end = clock();
            time_taken = (double)(end - start) / CLOCKS_PER_SEC;

            // 6. Publish Results and Visualization
            if (VIZ_MDOE_){
                // publishTriGridFieldGraph();
                // publishTriGridCorners();
                // publishPointClouds();
            }
            return;
        };

        TriGridIdx getTriGridIdx(const float& x_in, const float& y_in){
            TriGridIdx tgf_idx;
            int r_i = (x_in - tgf_min_x)/TGF_RESOLUTION_;
            int c_i = (y_in - tgf_min_y)/TGF_RESOLUTION_;
            int t_i = 0;
            double angle = atan2(y_in-(c_i*TGF_RESOLUTION_ + TGF_RESOLUTION_/2 + tgf_min_y), x_in-(r_i*TGF_RESOLUTION_ + TGF_RESOLUTION_/2 + tgf_min_x));

            if (angle>=(M_PI/4) && angle <(3*M_PI/4)){
                t_i = 1;
            } else if (angle>=(-M_PI/4) && angle <(M_PI/4)){
                t_i = 0;
            } else if (angle>=(-3*M_PI/4) && angle <(-M_PI/4)){
                t_i = 3;
            } else{
                t_i = 2;
            }
            tgf_idx.row = r_i;
            tgf_idx.col = c_i;
            tgf_idx.tri = t_i;
            return tgf_idx;
        }

        TriGridNode<PointType> getTriGridNode(const float& x_in, const float& y_in){
            TriGridNode<PointType> node;
            TriGridIdx node_idx = getTriGridIdx(x_in, y_in);
            node = trigrid_field_[node_idx.row][node_idx.col][node_idx.tri];
            return node;
        };

        TriGridNode<PointType> getTriGridNode(const TriGridIdx& tgf_idx){
            TriGridNode<PointType> node;
            node = trigrid_field_[tgf_idx.row][tgf_idx.col][tgf_idx.tri];
            return node;
        };

        bool is_traversable(const float& x_in, const float& y_in){
            TriGridNode<PointType> node = getTriGridNode(x_in, y_in);
            if (node.node_type == GROUND){
                return true;
            } else{
                return false;
            }
        };

        pcl::PointCloud<PointType> getObstaclePC(){
            pcl::PointCloud<PointType> cloud_obstacle;
            cloud_obstacle = ptCloud_tgfwise_obstacle_;
            return cloud_obstacle;
        };

    private:
        double tgf_max_x, tgf_max_y, tgf_min_x, tgf_min_y;
        double rows_, cols_;

        void initTriGridField(TriGridField<PointType>& tgf_in){
            // ROS_INFO("Initializing TriGridField...");

            tgf_max_x = MAX_RANGE_;
            tgf_max_y = MAX_RANGE_;

            tgf_min_x = -MAX_RANGE_;
            tgf_min_y = -MAX_RANGE_;

            rows_ = (int)(tgf_max_x - tgf_min_x) / TGF_RESOLUTION_;
            cols_ = (int)(tgf_max_y - tgf_min_y) / TGF_RESOLUTION_;
            empty_cloud_.clear();
            empty_cloud_.reserve(PTCLOUD_SIZE);
            
            // Set Node structure
            empty_trigrid_node_.node_type = UNKNOWN;
            empty_trigrid_node_.ptCloud.clear();
            empty_trigrid_node_.ptCloud.reserve(NODEWISE_PTCLOUDSIZE);

            empty_trigrid_node_.is_curr_data = false;
            empty_trigrid_node_.need_recheck = false;
            empty_trigrid_node_.is_visited = false;
            empty_trigrid_node_.is_rejection = false;

            empty_trigrid_node_.check_life = 10;
            empty_trigrid_node_.depth = -1;

            empty_trigrid_node_.normal;
            empty_trigrid_node_.mean_pt;
            empty_trigrid_node_.d = 0;
            
            empty_trigrid_node_.singular_values;
            empty_trigrid_node_.eigen_vectors;
            empty_trigrid_node_.weight = 0;

            empty_trigrid_node_.th_dist_d = 0;
            empty_trigrid_node_.th_outlier_d = 0;

            // Set TriGridField
            tgf_in.clear();
            std::vector<GridNode<PointType>> vec_gridnode;

            for (int i = 0; i < 4 ; i ++) 
                empty_grid_nodes_.emplace_back(empty_trigrid_node_);
                
            for (int i=0; i< cols_; i++){ vec_gridnode.emplace_back(empty_grid_nodes_);}
            for (int j=0; j< rows_; j++){ tgf_in.emplace_back(vec_gridnode);}

            return;
        };

        void initTriGridCorners(std::vector<std::vector<TriGridCorner>>& trigrid_corners_in,
                                std::vector<std::vector<TriGridCorner>>& trigrid_centers_in){
            // ROS_INFO("Initializing TriGridCorners...");

            // Set TriGridCorner
            empty_trigrid_corner_.x = empty_trigrid_corner_.y = 0.0;
            empty_trigrid_corner_.zs.clear();
            empty_trigrid_corner_.zs.reserve(8);
            empty_trigrid_corner_.weights.clear();
            empty_trigrid_corner_.weights.reserve(8);

            empty_trigrid_center_.x = empty_trigrid_center_.y = 0.0;
            empty_trigrid_center_.zs.clear();
            empty_trigrid_center_.zs.reserve(4);
            empty_trigrid_center_.weights.clear();
            empty_trigrid_center_.weights.reserve(4);

            trigrid_corners_in.clear();
            trigrid_centers_in.clear();

            // Set Corners and Centers
            std::vector<TriGridCorner> col_corners;
            std::vector<TriGridCorner> col_centers;
            for (int i=0; i< cols_; i++){
                col_corners.emplace_back(empty_trigrid_corner_);
                col_centers.emplace_back(empty_trigrid_center_);
            }
            col_corners.emplace_back(empty_trigrid_corner_);

            for (int j=0; j< rows_; j++){
                trigrid_corners_in.emplace_back(col_corners);
                trigrid_centers_in.emplace_back(col_centers);
            }
            trigrid_corners_in.emplace_back(col_corners);

            return;
        };

        void clearTriGridField(TriGridField<PointType> &tgf_in){
            // ROS_INFO("Clearing TriGridField...");

            for (int r_i = 0; r_i < rows_; r_i++){
            for (int c_i = 0; c_i < cols_; c_i++){
                tgf_in[r_i][c_i] = empty_grid_nodes_;
            }
            }
            return;
        };

        void clearTriGridCorners(std::vector<std::vector<TriGridCorner>>& trigrid_corners_in,
                                std::vector<std::vector<TriGridCorner>>& trigrid_centers_in){
            // ROS_INFO("Clearing TriGridCorners...");

            TriGridCorner tmp_corner = empty_trigrid_corner_;
            TriGridCorner tmp_center = empty_trigrid_center_;
            for (int r_i = 0; r_i < rows_+1; r_i++){
            for (int c_i = 0; c_i < cols_+1; c_i++){
                tmp_corner.x = (r_i)*TGF_RESOLUTION_+tgf_min_x; tmp_corner.y = (c_i)*TGF_RESOLUTION_+tgf_min_y;
                tmp_corner.zs.clear();
                tmp_corner.weights.clear();
                trigrid_corners_in[r_i][c_i] = tmp_corner;
                if (r_i < rows_ && c_i < cols_) {
                    tmp_center.x = (r_i+0.5)*TGF_RESOLUTION_+tgf_min_x; tmp_center.y = (c_i+0.5)*TGF_RESOLUTION_+tgf_min_y;
                    tmp_center.zs.clear();
                    tmp_center.weights.clear();
                    trigrid_centers_in[r_i][c_i] = tmp_center;
                }
            }
            }
            return;
        };
        
        double xy_2Dradius(double x, double y){
            return sqrt(x*x + y*y);
        };

        bool filterPoint(const PointType &pt_in){
            double xy_range = xy_2Dradius(pt_in.x, pt_in.y);
            if (xy_range >= MAX_RANGE_ || xy_range <= MIN_RANGE_) return true;

            return false;
        }

        void embedCloudToTriGridField(const pcl::PointCloud<PointType>& cloud_in, TriGridField<PointType>& tgf_out) {
            // ROS_INFO("Embedding PointCloud to TriGridField...");

            for (auto const &pt: cloud_in.points){
                if (filterPoint(pt)){
                    ptCloud_tgfwise_outliers_.points.push_back(pt);
                    continue;   
                }

                int r_i = (pt.x - tgf_min_x)/TGF_RESOLUTION_;
                int c_i = (pt.y - tgf_min_y)/TGF_RESOLUTION_;

                if (r_i < 0 || r_i >= rows_ || c_i < 0 || c_i >= cols_) {
                    ptCloud_tgfwise_outliers_.points.push_back(pt);
                    continue;
                }

                double angle = atan2(pt.y-(c_i*TGF_RESOLUTION_ + TGF_RESOLUTION_/2 + tgf_min_y), pt.x-(r_i*TGF_RESOLUTION_ + TGF_RESOLUTION_/2 + tgf_min_x));
                if (angle>=(M_PI/4) && angle <(3*M_PI/4)){
                    // left side
                    tgf_out[r_i][c_i][1].ptCloud.push_back(pt);
                    if(!tgf_out[r_i][c_i][1].is_curr_data) {tgf_out[r_i][c_i][1].is_curr_data = true;}
                } else if (angle>=(-M_PI/4) && angle <(M_PI/4)){
                    // upper side
                    tgf_out[r_i][c_i][0].ptCloud.push_back(pt);
                    if (!tgf_out[r_i][c_i][0].is_curr_data){tgf_out[r_i][c_i][0].is_curr_data = true;}
                    
                } else if (angle>=(-3*M_PI/4) && angle <(-M_PI/4)){
                    // right side
                    tgf_out[r_i][c_i][3].ptCloud.push_back(pt);
                    if (!tgf_out[r_i][c_i][3].is_curr_data) {tgf_out[r_i][c_i][3].is_curr_data = true;}
                } else{
                    // lower side
                    tgf_out[r_i][c_i][2].ptCloud.push_back(pt);
                    if (!tgf_out[r_i][c_i][2].is_curr_data) {tgf_out[r_i][c_i][2].is_curr_data = true;}
                }
            }

            return;
        };

        void extractInitialSeeds(const pcl::PointCloud<PointType>& p_sorted, pcl::PointCloud<PointType>& init_seeds){
            //function for uniform mode
            init_seeds.points.clear();

            // LPR is the mean of Low Point Representative
            double sum = 0;
            int cnt = 0;

            // Calculate the mean height value.
            for (int i=0; i< (int) p_sorted.points.size() && cnt<NUM_LRP_; i++){
                sum += p_sorted.points[i].z;
                cnt++;
            }

            double lpr_height = cnt!=0?sum/cnt:0;

            for(int i=0 ; i< (int) p_sorted.points.size() ; i++){
                if(p_sorted.points[i].z < lpr_height + TH_SEEDS_){
                    if (p_sorted.points[i].z < lpr_height-TH_OUTLIER_) continue;
                    init_seeds.points.push_back(p_sorted.points[i]);
                }
            }

            return;
        }

        void estimatePlanarModel(const pcl::PointCloud<PointType>& ground_in, TriGridNode<PointType>& node_out) {

            // function for uniform mode
            Eigen::Matrix3f cov_;
            Eigen::Vector4f pc_mean_;
            pcl::computeMeanAndCovarianceMatrix(ground_in, cov_, pc_mean_);

            // Singular Value Decomposition: SVD
            Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);

            // Use the least singular vector as normal
            node_out.eigen_vectors = svd.matrixU();
            if (node_out.eigen_vectors.col(2)(2,0)<0) {
                node_out.eigen_vectors.col(0)*= -1;
                node_out.eigen_vectors.col(2)*= -1;
            }
            node_out.normal = node_out.eigen_vectors.col(2);
            node_out.singular_values = svd.singularValues();

            // mean ground seeds value
            node_out.mean_pt = pc_mean_.head<3>();

            // according to noraml.T*[x,y,z] = -d
            node_out.d = -(node_out.normal.transpose()*node_out.mean_pt)(0,0);

            // set distance theshold to 'th_dist - d'
            node_out.th_dist_d = TH_DIST_ - node_out.d;
            node_out.th_outlier_d = -node_out.d - TH_OUTLIER_;

            return;
        }    

        void modelPCAbasedTerrain(TriGridNode<PointType>& node_in) {
            // Initailization
            if (!ptCloud_nodewise_ground_.empty()) ptCloud_nodewise_ground_.clear();
            
            // Tri Grid Initialization
            // When to initialize the planar model, we don't have prior. so outlier is removed in heuristic parameter.
            pcl::PointCloud<PointType> sort_ptCloud = node_in.ptCloud;

            // sort in z-coordinate
            sort(sort_ptCloud.points.begin(), sort_ptCloud.end(), point_z_cmp<PointType>);

            // Set init seeds
            extractInitialSeeds(sort_ptCloud, ptCloud_nodewise_ground_);
            
            Eigen::MatrixXf points(sort_ptCloud.points.size(),3);
            int j = 0;
            for (auto& p:sort_ptCloud.points){
                points.row(j++)<<p.x, p.y, p.z;
            }
            // Extract Ground
            for (int i =0; i < NUM_ITER_; i++){
                estimatePlanarModel(ptCloud_nodewise_ground_, node_in);
                if(ptCloud_nodewise_ground_.size() < 3){
    
                    node_in.node_type = NONGROUND;
                    break;
                }
                ptCloud_nodewise_ground_.clear();
                // threshold filter
                Eigen::VectorXf result = points*node_in.normal;
                for (int r = 0; r<result.rows(); r++){
                    if (i < NUM_ITER_-1){
                        if (result[r]<node_in.th_dist_d){
                            ptCloud_nodewise_ground_.push_back(sort_ptCloud.points[r]);
                        }
                    } else {
                        // Final interation
                        if (node_in.normal(2,0) < TH_NORMAL_ ){
                            node_in.node_type = NONGROUND;
                        } else {
                            node_in.node_type = GROUND;
                        }
                    }
                }
            }

            return;
        }

        double calcNodeWeight(const TriGridNode<PointType>& node_in){
            double weight = 0;
            
            // weight = (node_in.singular_values[0]/node_in.singular_values[2] + node_in.singular_values[1]/node_in.singular_values[2])/(node_in.singular_values[0]/node_in.singular_values[1]);
            weight = (node_in.singular_values[0] + node_in.singular_values[1])*node_in.singular_values[1]/(node_in.singular_values[0]*node_in.singular_values[2]+0.001);

            return weight;
        }

        void modelNodeWiseTerrain(TriGridField<PointType>& tgf_in) {
            // ROS_INFO("Node-wise Terrain Modeling...");

            for (int r_i = 0; r_i < rows_; r_i++){
            for (int c_i = 0; c_i < cols_; c_i++){
            for (int s_i = 0; s_i < 4; s_i++){
                if (tgf_in[r_i][c_i][s_i].is_curr_data){
                    if (tgf_in[r_i][c_i][s_i].ptCloud.size() < NUM_MIN_POINTS_){
                        tgf_in[r_i][c_i][s_i].node_type = UNKNOWN;
                        continue;                    
                    } else {
                        modelPCAbasedTerrain(tgf_in[r_i][c_i][s_i]);
                        if (tgf_in[r_i][c_i][s_i].node_type == GROUND){ tgf_in[r_i][c_i][s_i].weight = calcNodeWeight(tgf_in[r_i][c_i][s_i]); }
                    }
                }
            }
            }
            }

            return;
        };

        void findDominantNode(const TriGridField<PointType>& tgf_in, TriGridIdx& node_idx_out) {
            // Find the dominant node
            ROS_INFO("Find the dominant node...");
            TriGridIdx max_tri_idx;
            TriGridIdx ego_idx;
            ego_idx.row = (int)((0-tgf_min_x)/TGF_RESOLUTION_);
            ego_idx.col = (int)((0-tgf_min_y)/TGF_RESOLUTION_);
            ego_idx.tri = 0;
            
            max_tri_idx = ego_idx;
            for (int r_i = ego_idx.row - 2; r_i < ego_idx.row + 2; r_i++){
            for (int c_i = ego_idx.col - 2; c_i < ego_idx.col + 2; c_i++){
            for (int s_i = 0; s_i < 4; s_i++){
                if (tgf_in[r_i][c_i][s_i].is_curr_data){
                    if (tgf_in[r_i][c_i][s_i].node_type == GROUND){
                        if (tgf_in[r_i][c_i][s_i].weight > tgf_in[max_tri_idx.row][max_tri_idx.row][max_tri_idx.tri].weight){
                            max_tri_idx.row = r_i;
                            max_tri_idx.col = c_i;
                            max_tri_idx.tri = s_i;
                        }
                    }
                }
            }
            }    
            }
            node_idx_out = max_tri_idx;
            return;
        };

        void searchNeighborNodes(const TriGridIdx &cur_idx, std::vector<TriGridIdx> &neighbor_idxs) {
            neighbor_idxs.clear();
            neighbor_idxs.reserve(14);
            int r_i = cur_idx.row;
            int c_i = cur_idx.col;
            int s_i = cur_idx.tri;

            std::vector<TriGridIdx> tmp_neighbors;
            tmp_neighbors.clear();
            tmp_neighbors.reserve(14);
            
            TriGridIdx neighbor_idx;
            for (int s_i = 0; s_i < 4 ; s_i++){
                if (s_i == cur_idx.tri) continue;

                neighbor_idx = cur_idx;
                neighbor_idx.tri = s_i;
                tmp_neighbors.push_back(neighbor_idx);
            }

            switch (s_i) {
                case 0:
                    tmp_neighbors.push_back({r_i+1, c_i+1, 2});
                    tmp_neighbors.push_back({r_i+1, c_i+1, 3});
                    tmp_neighbors.push_back({r_i+1, c_i  , 1});
                    tmp_neighbors.push_back({r_i+1, c_i  , 2});
                    tmp_neighbors.push_back({r_i+1, c_i  , 3});
                    tmp_neighbors.push_back({r_i+1, c_i-1, 1});
                    tmp_neighbors.push_back({r_i+1, c_i-1, 2});
                    tmp_neighbors.push_back({r_i  , c_i+1, 0});
                    tmp_neighbors.push_back({r_i  , c_i+1, 3});
                    tmp_neighbors.push_back({r_i  , c_i-1, 0});
                    tmp_neighbors.push_back({r_i  , c_i-1, 1});
                    break;
                case 1:
                    tmp_neighbors.push_back({r_i+1, c_i+1, 2});
                    tmp_neighbors.push_back({r_i+1, c_i+1, 3});
                    tmp_neighbors.push_back({r_i+1, c_i  , 1});
                    tmp_neighbors.push_back({r_i+1, c_i  , 2});
                    tmp_neighbors.push_back({r_i  , c_i+1, 0});
                    tmp_neighbors.push_back({r_i  , c_i+1, 2});
                    tmp_neighbors.push_back({r_i  , c_i+1, 3});
                    tmp_neighbors.push_back({r_i-1, c_i+1, 0});
                    tmp_neighbors.push_back({r_i-1, c_i+1, 3});
                    tmp_neighbors.push_back({r_i-1, c_i+1, 0});
                    tmp_neighbors.push_back({r_i-1, c_i  , 1});
                    break;
                case 2:
                    tmp_neighbors.push_back({r_i  , c_i+1, 2});
                    tmp_neighbors.push_back({r_i  , c_i+1, 3});
                    tmp_neighbors.push_back({r_i  , c_i-1, 1});
                    tmp_neighbors.push_back({r_i  , c_i-1, 2});
                    tmp_neighbors.push_back({r_i-1, c_i+1, 0});
                    tmp_neighbors.push_back({r_i-1, c_i+1, 3});
                    tmp_neighbors.push_back({r_i-1, c_i  , 0});
                    tmp_neighbors.push_back({r_i-1, c_i  , 1});
                    tmp_neighbors.push_back({r_i-1, c_i  , 3});
                    tmp_neighbors.push_back({r_i-1, c_i-1, 0});
                    tmp_neighbors.push_back({r_i-1, c_i-1, 1});
                    break;
                case 3:
                    tmp_neighbors.push_back({r_i+1, c_i  , 2});
                    tmp_neighbors.push_back({r_i+1, c_i  , 3});
                    tmp_neighbors.push_back({r_i+1, c_i-1, 1});
                    tmp_neighbors.push_back({r_i+1, c_i-1, 2});
                    tmp_neighbors.push_back({r_i  , c_i-1, 0});
                    tmp_neighbors.push_back({r_i  , c_i-1, 1});
                    tmp_neighbors.push_back({r_i  , c_i-1, 2});
                    tmp_neighbors.push_back({r_i-1, c_i  , 0});
                    tmp_neighbors.push_back({r_i-1, c_i  , 3});
                    tmp_neighbors.push_back({r_i-1, c_i-1, 0});
                    tmp_neighbors.push_back({r_i-1, c_i-1, 1});
                    break;
                default:
                    break;
            }

            for (int n_i = 0 ; n_i < (int) tmp_neighbors.size() ; n_i++) {

                if (tmp_neighbors[n_i].row >= rows_ || tmp_neighbors[n_i].row < 0) {
                    continue;
                }

                if (tmp_neighbors[n_i].col >= cols_ || tmp_neighbors[n_i].col < 0) {
                    continue;
                }

                neighbor_idxs.push_back(tmp_neighbors[n_i]);
            }
        }

        void searchAdjacentNodes(const TriGridIdx &cur_idx, std::vector<TriGridIdx> &adjacent_idxs) {
            adjacent_idxs.clear();
            adjacent_idxs.reserve(3);
            int r_i = cur_idx.row;
            int c_i = cur_idx.col;
            int s_i = cur_idx.tri;

            std::vector<TriGridIdx> tmp_neighbors;
            tmp_neighbors.clear();
            tmp_neighbors.reserve(3);
            
            TriGridIdx neighbor_idx;

            switch (s_i) {
                case 0:
                    tmp_neighbors.push_back({r_i+1, c_i, 2});
                    tmp_neighbors.push_back({r_i  , c_i, 3});
                    tmp_neighbors.push_back({r_i  , c_i, 1});
     
                    break;
                case 1:
                    tmp_neighbors.push_back({r_i, c_i+1, 3});
                    tmp_neighbors.push_back({r_i, c_i  , 0});
                    tmp_neighbors.push_back({r_i, c_i  , 2});                    
                    break;
                case 2:
                    tmp_neighbors.push_back({r_i-1, c_i, 0});
                    tmp_neighbors.push_back({r_i  , c_i, 1});
                    tmp_neighbors.push_back({r_i  , c_i, 3});
                    break;
                case 3:
                    tmp_neighbors.push_back({r_i, c_i-1, 1});
                    tmp_neighbors.push_back({r_i, c_i  , 2});
                    tmp_neighbors.push_back({r_i, c_i  , 0});
                    break;
                default:
                    break;
            }

            for (int n_i = 0 ; n_i < (int) tmp_neighbors.size() ; n_i++) {

                if (tmp_neighbors[n_i].row >= rows_ || tmp_neighbors[n_i].row < 0) {
                    continue;
                }

                if (tmp_neighbors[n_i].col >= cols_ || tmp_neighbors[n_i].col < 0) {
                    continue;
                }

                adjacent_idxs.push_back(tmp_neighbors[n_i]);
            }
        }

        bool LocalConvecityConcavity(const TriGridField<PointType> &tgf, const TriGridIdx &cur_node_idx, const TriGridIdx &neighbor_idx, 
                                    double & thr_local_normal, double & thr_local_dist) {
            TriGridNode<PointType> current_node = tgf[cur_node_idx.row][cur_node_idx.col][cur_node_idx.tri];
            TriGridNode<PointType> neighbor_node = tgf[neighbor_idx.row][neighbor_idx.col][neighbor_idx.tri];

            Eigen::Vector3f normal_src = current_node.normal; 
            Eigen::Vector3f normal_tgt = neighbor_node.normal; 
            Eigen::Vector3f meanPt_diff_s2t = neighbor_node.mean_pt - current_node.mean_pt;

            double diff_norm = meanPt_diff_s2t.norm();
            double dist_s2t = normal_src.dot(meanPt_diff_s2t);
            double dist_t2s = normal_tgt.dot(-meanPt_diff_s2t);

            double normal_similarity = normal_src.dot(normal_tgt);
            double TH_NORMAL_cos_similarity = sin(diff_norm*thr_local_normal);
            if ((normal_similarity < (1-TH_NORMAL_cos_similarity))) {
                return false;
            }

            double TH_DIST_to_planar = diff_norm*sin(thr_local_dist);
            if ( (abs(dist_s2t) > TH_DIST_to_planar || abs(dist_t2s) > TH_DIST_to_planar) ) {
                return false;
            }

            return true;
        }

        void BreadthFirstTraversableGraphSearch(TriGridField<PointType>& tgf_in) {

            // Find the dominant node
            std::queue<TriGridIdx> searching_idx_queue;
            TriGridIdx dominant_node_idx;
            findDominantNode(tgf_in, dominant_node_idx);
            tgf_in[dominant_node_idx.row][dominant_node_idx.col][dominant_node_idx.tri].is_visited = true;
            tgf_in[dominant_node_idx.row][dominant_node_idx.col][dominant_node_idx.tri].depth = 0;
            tgf_in[dominant_node_idx.row][dominant_node_idx.col][dominant_node_idx.tri].node_type = GROUND;

            searching_idx_queue.push(dominant_node_idx);

            double max_planar_height = 0;
            trigrid_edges_.clear();
            trigrid_edges_.reserve(rows_*cols_*4);
            TriGridEdge cur_edge;
            TriGridIdx current_node_idx;
            while (!searching_idx_queue.empty()){
                // set current node
                current_node_idx = searching_idx_queue.front();
                searching_idx_queue.pop();

                // search the neighbor nodes
                std::vector<TriGridIdx> neighbor_idxs;
                searchNeighborNodes(current_node_idx, neighbor_idxs);
                
                // set the traversable edges
                for (int i = 0; i < (int) neighbor_idxs.size(); i++){
                    // if the neighbor node is traversable, add it to the queue

                    TriGridIdx n_i = neighbor_idxs[i];


                    if (tgf_in[n_i.row][n_i.col][n_i.tri].depth >=0){
                        continue;
                    }

                    if (tgf_in[n_i.row][n_i.col][n_i.tri].is_visited) {
                        if (!tgf_in[n_i.row][n_i.col][n_i.tri].need_recheck){
                            continue;
                        } else {
                            if (tgf_in[n_i.row][n_i.col][n_i.tri].check_life <= 0){
                                continue;
                            }
                        }
                        continue;
                    } else {
                        if (tgf_in[n_i.row][n_i.col][n_i.tri].node_type != GROUND) {
                        continue;
                        }
                    }

                    tgf_in[n_i.row][n_i.col][n_i.tri].is_visited =true;
                    
                    if (!LocalConvecityConcavity(tgf_in, current_node_idx, n_i, TH_LCC_NORMAL_SIMILARITY_, TH_LCC_PLANAR_MODEL_DIST_)) {
                        tgf_in[n_i.row][n_i.col][n_i.tri].is_rejection = true;
                        tgf_in[n_i.row][n_i.col][n_i.tri].node_type = NONGROUND;

                        if(tgf_in[n_i.row][n_i.col][n_i.tri].check_life > 0) {
                            tgf_in[n_i.row][n_i.col][n_i.tri].check_life -=1;
                            tgf_in[n_i.row][n_i.col][n_i.tri].need_recheck = true;
                        } else {
                            tgf_in[n_i.row][n_i.col][n_i.tri].need_recheck = false;
                        }
                        continue;
                    }

                    if (max_planar_height < tgf_in[n_i.row][n_i.col][n_i.tri].mean_pt[2]) max_planar_height = tgf_in[n_i.row][n_i.col][n_i.tri].mean_pt[2];

                    tgf_in[n_i.row][n_i.col][n_i.tri].node_type = GROUND;
                    tgf_in[n_i.row][n_i.col][n_i.tri].is_rejection = false;
                    tgf_in[n_i.row][n_i.col][n_i.tri].depth = tgf_in[current_node_idx.row][current_node_idx.col][current_node_idx.tri].depth + 1;

                    if (VIZ_MDOE_){
                        cur_edge.Pair.first = current_node_idx;
                        cur_edge.Pair.second = n_i;
                        cur_edge.is_traversable = true;
                        trigrid_edges_.push_back(cur_edge);
                    }

                    searching_idx_queue.push(n_i);
                }

                if (searching_idx_queue.empty()){
                    // set the new dominant node
                    for (int r_i = 0; r_i < rows_; r_i++) {
                    for (int c_i = 0; c_i < cols_; c_i++) {
                    for (int s_i = 0; s_i < (int) tgf_in[r_i][c_i].size() ; s_i++){
                        if (tgf_in[r_i][c_i][s_i].is_visited) { continue; }

                        if (tgf_in[r_i][c_i][s_i].node_type != GROUND ) { continue; }

                        // if (tgf_in[r_i][c_i][s_i].mean_pt[2] >= max_planar_height+1) { continue; }

                        if (tgf_in[r_i][c_i][s_i].depth >= 0) { continue; }

                        // if (tgf_in[r_i][c_i][s_i].weight > 5*TH_WEIGHT_){
                            tgf_in[r_i][c_i][s_i].depth = 0;
                            tgf_in[r_i][c_i][s_i].is_visited = true;

                            TriGridIdx new_dominant_idx = {r_i, c_i, s_i};
                            searching_idx_queue.push(new_dominant_idx);
                        // }
                    }
                    }
                    }
                }
            }
            return;
        };


        double getCornerWeight(const TriGridNode<PointType>& node_in, const pcl::PointXYZ &tgt_corner){
            double xy_dist = sqrt( (node_in.mean_pt[0]-tgt_corner.x)*(node_in.mean_pt[0]-tgt_corner.x)+(node_in.mean_pt[1]-tgt_corner.y)*(node_in.mean_pt[1]-tgt_corner.y) );
            return (node_in.weight/xy_dist);
        }

        void setTGFCornersCenters(const TriGridField<PointType>& tgf_in,
                                std::vector<std::vector<TriGridCorner>>& trigrid_corners_out,
                                std::vector<std::vector<TriGridCorner>>& trigrid_centers_out) {
            pcl::PointXYZ corner_TL, corner_BL, corner_BR, corner_TR, corner_C;

            for (int r_i = 0; r_i<rows_; r_i++){
            for (int c_i = 0; c_i<cols_; c_i++){
                corner_TL.x = trigrid_corners_out[r_i+1][c_i+1].x; corner_TL.y = trigrid_corners_out[r_i+1][c_i+1].y;   // LT
                corner_BL.x = trigrid_corners_out[r_i][c_i+1].x;   corner_BL.y = trigrid_corners_out[r_i][c_i+1].y;     // LL
                corner_BR.x = trigrid_corners_out[r_i][c_i].x;     corner_BR.y = trigrid_corners_out[r_i][c_i].y;       // RL
                corner_TR.x = trigrid_corners_out[r_i+1][c_i].x;   corner_TR.y = trigrid_corners_out[r_i+1][c_i].y;     // RT
                corner_C.x = trigrid_centers_out[r_i][c_i].x;    corner_C.y = trigrid_centers_out[r_i][c_i].y;       // Center

                for (int s_i = 0; s_i< (int) tgf_in[r_i][c_i].size();s_i++){
                    if (tgf_in[r_i][c_i][s_i].node_type != GROUND) { continue; }
                    if (tgf_in[r_i][c_i][s_i].is_rejection) { continue; }
                    if (tgf_in[r_i][c_i][s_i].depth == -1) { continue; }

                    switch(s_i){
                        case 0: // upper Tri-grid bin
                            // RT / LT / C
                            trigrid_corners_out[r_i+1][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_TR.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_TR.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i+1][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_TR));

                            trigrid_corners_out[r_i+1][c_i+1].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_TL.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_TL.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i+1][c_i+1].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_TL));

                            trigrid_centers_out[r_i][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_C.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_C.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_centers_out[r_i][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_C));

                            break;
                        case 1: // left Tri-grid bin
                            // LT / LL / C
                            trigrid_corners_out[r_i+1][c_i+1].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_TL.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_TL.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i+1][c_i+1].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_TL));

                            trigrid_corners_out[r_i][c_i+1].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_BL.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_BL.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i][c_i+1].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_BL));

                            trigrid_centers_out[r_i][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_C.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_C.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_centers_out[r_i][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_C));

                            break;
                        case 2: // lower Tri-grid bin
                            // LL / RL / C
                            trigrid_corners_out[r_i][c_i+1].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_BL.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_BL.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i][c_i+1].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_BL));

                            trigrid_corners_out[r_i][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_BR.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_BR.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_BR));

                            trigrid_centers_out[r_i][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_C.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_C.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_centers_out[r_i][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_C));
                            
                            break;
                        case 3: // right Tri-grid bin
                            // RL / RT / C
                            trigrid_corners_out[r_i][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_BR.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_BR.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_BR));

                            trigrid_corners_out[r_i+1][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_TR.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_TR.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i+1][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_TR));

                            trigrid_centers_out[r_i][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_C.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_C.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_centers_out[r_i][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_C));
                            
                            break;
                        default:
                            break;
                    }
                }
            }
            }
            return;
        };
        
        TriGridCorner getMeanCorner(const TriGridCorner &corners_in){
            // get the mean of the corners

            TriGridCorner corners_out;
            corners_out.x = corners_in.x;
            corners_out.y = corners_in.y;
            corners_out.zs.clear();
            corners_out.weights.clear();

            double weighted_sum_z = 0.0;
            double sum_w = 0.0;
            for (int i = 0; i < (int) corners_in.zs.size(); i++){
                weighted_sum_z += corners_in.zs[i]*corners_in.weights[i];
                sum_w += corners_in.weights[i];
            }

            corners_out.zs.push_back(weighted_sum_z/sum_w);
            corners_out.weights.push_back(sum_w);

            return corners_out;
        }

        void updateTGFCornersCenters(std::vector<std::vector<TriGridCorner>>& trigrid_corners_out,
                                    std::vector<std::vector<TriGridCorner>>& trigrid_centers_out) {

            // update corners
            TriGridCorner updated_corner = empty_trigrid_corner_;
            for (int r_i = 0; r_i < rows_ +1; r_i++) {
            for (int c_i = 0; c_i < cols_ +1; c_i++) {
                if (trigrid_corners_out[r_i][c_i].zs.size() > 0 && trigrid_corners_out[r_i][c_i].weights.size() > 0) {
                    updated_corner = getMeanCorner(trigrid_corners_out[r_i][c_i]);
                    trigrid_corners_out[r_i][c_i] = updated_corner;
                } else {
                    trigrid_corners_out[r_i][c_i].zs.clear();
                    trigrid_corners_out[r_i][c_i].weights.clear();
                }
            }
            }        

            // update centers
            TriGridCorner updated_center = empty_trigrid_center_;
            for (int r_i = 0; r_i < rows_; r_i++) {
            for (int c_i = 0; c_i < cols_; c_i++) {
                if (trigrid_centers_out[r_i][c_i].zs.size() > 0 && trigrid_centers_out[r_i][c_i].weights.size() > 0) {
                    updated_center = getMeanCorner(trigrid_centers_out[r_i][c_i]);
                    trigrid_centers_out[r_i][c_i] = updated_center;
                    // trigrid_centers_out[r_i][c_i].z = get_mean(trigrid_centers_out[r_i][c_i].zs,trigrid_centers_out[r_i][c_i].weights);
                } else {
                    trigrid_centers_out[r_i][c_i].zs.clear();
                    trigrid_centers_out[r_i][c_i].weights.clear();
                }
            }
            }

            return;
        };

        Eigen::Vector3f convertCornerToEigen(TriGridCorner &corner_in) {
            Eigen::Vector3f corner_out;
            if (corner_in.zs.size() != corner_in.weights.size()){
                ROS_WARN("ERROR in corners");
            }
            corner_out[0] = corner_in.x;
            corner_out[1] = corner_in.y;
            corner_out[2] = corner_in.zs[0];
            return corner_out;
        };

        void revertTraversableNodes(std::vector<std::vector<TriGridCorner>>& trigrid_corners_in,
                                    std::vector<std::vector<TriGridCorner>>& trigrid_centers_in, 
                                    TriGridField<PointType>& tgf_out) {
            Eigen::Vector3f refined_corner_1, refined_corner_2, refined_center;
            for (int r_i = 0; r_i < rows_; r_i++) {
            for (int c_i = 0; c_i < cols_; c_i++) {
            for (int s_i = 0; s_i < (int) tgf_out[r_i][c_i].size(); s_i++) {
                // set the corners for the current trigrid node
                switch (s_i)
                {
                case 0:
                    if ( trigrid_corners_in[r_i+1][c_i].zs.size()==0 || trigrid_corners_in[r_i+1][c_i+1].zs.size()==0  || trigrid_centers_in[r_i][c_i].zs.size()==0 ){
                        if (tgf_out[r_i][c_i][s_i].node_type != NONGROUND){
                            tgf_out[r_i][c_i][s_i].node_type = UNKNOWN;
                        }
                        continue;
                    }
                    refined_corner_1 = convertCornerToEigen(trigrid_corners_in[r_i+1][c_i]);
                    refined_corner_2 = convertCornerToEigen(trigrid_corners_in[r_i+1][c_i+1]);
                    refined_center = convertCornerToEigen(trigrid_centers_in[r_i][c_i]);
                    break;
                case 1:
                    if ( trigrid_corners_in[r_i+1][c_i+1].zs.size()==0 || trigrid_corners_in[r_i][c_i+1].zs.size()==0  || trigrid_centers_in[r_i][c_i].zs.size()==0 ){
                        if (tgf_out[r_i][c_i][s_i].node_type != NONGROUND){
                            tgf_out[r_i][c_i][s_i].node_type = UNKNOWN;
                        }
                        continue;
                    }
                    refined_corner_1 = convertCornerToEigen(trigrid_corners_in[r_i+1][c_i+1]);
                    refined_corner_2 = convertCornerToEigen(trigrid_corners_in[r_i][c_i+1]);
                    refined_center = convertCornerToEigen(trigrid_centers_in[r_i][c_i]);    
                    break;
                case 2:
                    if ( trigrid_corners_in[r_i][c_i+1].zs.size()==0 || trigrid_corners_in[r_i][c_i].zs.size()==0  || trigrid_centers_in[r_i][c_i].zs.size()==0 ){
                        if (tgf_out[r_i][c_i][s_i].node_type != NONGROUND){
                            tgf_out[r_i][c_i][s_i].node_type = UNKNOWN;
                        }
                        continue;
                    }
                    refined_corner_1 = convertCornerToEigen(trigrid_corners_in[r_i][c_i+1]);
                    refined_corner_2 = convertCornerToEigen(trigrid_corners_in[r_i][c_i]);
                    refined_center = convertCornerToEigen(trigrid_centers_in[r_i][c_i]);
                    break;
                case 3:
                    if ( trigrid_corners_in[r_i][c_i].zs.size()==0 || trigrid_corners_in[r_i+1][c_i].zs.size()==0  || trigrid_centers_in[r_i][c_i].zs.size()==0 ){
                        if (tgf_out[r_i][c_i][s_i].node_type != NONGROUND){
                            tgf_out[r_i][c_i][s_i].node_type = UNKNOWN;
                        }
                        continue;
                    }
                    refined_corner_1 = convertCornerToEigen(trigrid_corners_in[r_i][c_i]);
                    refined_corner_2 = convertCornerToEigen(trigrid_corners_in[r_i+1][c_i]);
                    refined_center = convertCornerToEigen(trigrid_centers_in[r_i][c_i]);
                    break;
                default:
                    ROS_ERROR("WRONG tri-grid indexing");
                    break;
                }

                // calculate the refined planar model in the node
                Eigen::Vector3f udpated_normal = (refined_corner_1-refined_center).cross(refined_corner_2-refined_center);
                udpated_normal /= udpated_normal.norm();
                // if (udpated_normal[2] < 0){
                //     std::cout << "Origin normal: " << tgf_out[r_i][c_i][s_i].normal << std::endl;
                //     std::cout << "Update normal: " << udpated_normal << std::endl;
                // }
                if (udpated_normal(2,0) < TH_NORMAL_ ){   // non-planar
                    tgf_out[r_i][c_i][s_i].normal = udpated_normal;
                    tgf_out[r_i][c_i][s_i].node_type = NONGROUND;
                } else {    
                    // planar
                    Eigen::Vector3f updated_mean_pt;
                    updated_mean_pt[0] = (refined_corner_1[0] + refined_corner_2[0] + refined_center[0])/3;
                    updated_mean_pt[1] = (refined_corner_1[1] + refined_corner_2[1] + refined_center[1])/3;
                    updated_mean_pt[2] = (refined_corner_1[2] + refined_corner_2[2] + refined_center[2])/3;

                    tgf_out[r_i][c_i][s_i].normal = udpated_normal;
                    tgf_out[r_i][c_i][s_i].mean_pt = updated_mean_pt;
                    tgf_out[r_i][c_i][s_i].d = -(udpated_normal.dot(updated_mean_pt));
                    tgf_out[r_i][c_i][s_i].th_dist_d = TH_DIST_ - tgf_out[r_i][c_i][s_i].d;
                    tgf_out[r_i][c_i][s_i].th_outlier_d = -TH_OUTLIER_ - tgf_out[r_i][c_i][s_i].d;

                    tgf_out[r_i][c_i][s_i].node_type = GROUND;
                }
            }
            }
            }

            return;
        };

        void fitTGFWiseTraversableTerrainModel(TriGridField<PointType>& tgf,
                                            std::vector<std::vector<TriGridCorner>>& trigrid_corners,
                                            std::vector<std::vector<TriGridCorner>>& trigrid_centers) {

            updateTGFCornersCenters(trigrid_corners, trigrid_centers);

            revertTraversableNodes(trigrid_corners, trigrid_centers, tgf);
            
            return;
        };

        void segmentNodeGround(const TriGridNode<PointType>& node_in,
                                pcl::PointCloud<PointType>& node_ground_out,
                                pcl::PointCloud<PointType>& node_nonground_out,
                                pcl::PointCloud<PointType>& node_obstacle_out,
                                pcl::PointCloud<PointType>& node_outlier_out) {
            node_ground_out.clear();
            node_nonground_out.clear();
            node_obstacle_out.clear();
            node_outlier_out.clear();

            // segment ground
            Eigen::MatrixXf points(node_in.ptCloud.points.size(),3);
            int j = 0; 
            for (auto& p:node_in.ptCloud.points){
                points.row(j++)<<p.x, p.y, p.z;
            }

            Eigen::VectorXf result = points*node_in.normal;
            for (int r = 0; r<result.rows(); r++){
                if (result[r]<node_in.th_dist_d){
                    if (result[r]<node_in.th_outlier_d){
                        node_outlier_out.push_back(node_in.ptCloud.points[r]);
                    } else {
                        node_ground_out.push_back(node_in.ptCloud.points[r]);
                    }
                } else {
                    node_nonground_out.push_back(node_in.ptCloud.points[r]);
                    if (result[r]<TH_OBSTACLE_HEIGHT_ - node_in.d){
                        node_obstacle_out.push_back(node_in.ptCloud.points[r]);
                        node_obstacle_out.points.back().intensity = result[r] + node_in.d;
                    }
                }
            }

            return;
        }

        void segmentTGFGround(const TriGridField<PointType>& tgf_in, 
                        pcl::PointCloud<PointType>& ground_cloud_out, 
                        pcl::PointCloud<PointType>& nonground_cloud_out,
                        pcl::PointCloud<PointType>& obstacle_cloud_out,
                        pcl::PointCloud<PointType>& outlier_cloud_out) {
            ground_cloud_out.clear();
            nonground_cloud_out.clear();
            obstacle_cloud_out.clear();

            for (int r_i = 0; r_i < rows_; r_i++){
            for (int c_i = 0; c_i < cols_; c_i++){
            for (int s_i = 0; s_i < tgf_in[r_i][c_i].size(); s_i++) {
                if (!tgf_in[r_i][c_i][s_i].is_curr_data) {
                    continue;
                }
                if (tgf_in[r_i][c_i][s_i].node_type == GROUND) {
                    segmentNodeGround(tgf_in[r_i][c_i][s_i], ptCloud_nodewise_ground_, ptCloud_nodewise_nonground_, ptCloud_nodewise_obstacle_, ptCloud_nodewise_outliers_);
                } else {
                    ptCloud_nodewise_nonground_ = tgf_in[r_i][c_i][s_i].ptCloud;
                    ptCloud_nodewise_obstacle_ = tgf_in[r_i][c_i][s_i].ptCloud;
                }
                ground_cloud_out += ptCloud_nodewise_ground_;
                nonground_cloud_out += ptCloud_nodewise_nonground_;
                outlier_cloud_out += ptCloud_nodewise_outliers_;
                obstacle_cloud_out += ptCloud_nodewise_obstacle_;
            }
            }
            }

            return;
        };

        void segmentTGFGround_developing(const TriGridField<PointType>& tgf_in, 
                        pcl::PointCloud<PointType>& ground_cloud_out, 
                        pcl::PointCloud<PointType>& nonground_cloud_out,
                        pcl::PointCloud<PointType>& obstacle_cloud_out,
                        pcl::PointCloud<PointType>& outlier_cloud_out) {
            ground_cloud_out.clear();
            nonground_cloud_out.clear();
            obstacle_cloud_out.clear();
            TriGridIdx curr_tgf_idx;
            std::vector<TriGridIdx> adj_idx_vec;
            pcl::PointCloud<PointType> outlier_tmp;
            outlier_tmp.clear();
            outlier_tmp.reserve(NODEWISE_PTCLOUDSIZE);
            for (int r_i = 0; r_i < rows_; r_i++){
            for (int c_i = 0; c_i < cols_; c_i++){
            for (int s_i = 0; s_i < (int) tgf_in[r_i][c_i].size(); s_i++) {
                if (!tgf_in[r_i][c_i][s_i].is_curr_data) {
                    continue;
                }
                if (tgf_in[r_i][c_i][s_i].node_type == UNKNOWN){
                    continue;
                }
                if (tgf_in[r_i][c_i][s_i].node_type == GROUND) {
                    segmentNodeGround(tgf_in[r_i][c_i][s_i], ptCloud_nodewise_ground_, ptCloud_nodewise_nonground_, ptCloud_nodewise_obstacle_, ptCloud_nodewise_outliers_);
                } else {
                    curr_tgf_idx.row = r_i;
                    curr_tgf_idx.col = c_i;
                    curr_tgf_idx.tri = s_i;
                    
                    searchAdjacentNodes(curr_tgf_idx, adj_idx_vec);
                    if (adj_idx_vec.empty()){
                        ptCloud_nodewise_nonground_ = tgf_in[r_i][c_i][s_i].ptCloud;
                        ptCloud_nodewise_obstacle_ = tgf_in[r_i][c_i][s_i].ptCloud;
                    } else {
                        TriGridIdx highest_adj_tri_idx;
                        double highest_weight = 0;
                        bool is_adjacent = false;
                        for (int adj_i = 0; adj_i<adj_idx_vec.size() ; adj_i++){
                            if (getTriGridNode(adj_idx_vec[adj_i]).weight < TH_WEIGHT_) {
                                continue;
                            }
                            is_adjacent = true;
                            if (getTriGridNode(adj_idx_vec[adj_i]).weight  > highest_weight) {
                                highest_weight = getTriGridNode(adj_idx_vec[adj_i]).weight;
                                highest_adj_tri_idx = adj_idx_vec[adj_i];
                            }
                        }

                        if (is_adjacent){
                            TriGridNode<PointType> tmp_node = getTriGridNode(highest_adj_tri_idx);
                            tmp_node.ptCloud = getTriGridNode(curr_tgf_idx).ptCloud;
                            segmentNodeGround(tmp_node, ptCloud_nodewise_ground_, ptCloud_nodewise_nonground_, ptCloud_nodewise_obstacle_, ptCloud_nodewise_outliers_);
                        } else {
                            ptCloud_nodewise_nonground_ = tgf_in[r_i][c_i][s_i].ptCloud;
                            ptCloud_nodewise_obstacle_ = tgf_in[r_i][c_i][s_i].ptCloud;
                        }
                    }
                }
                ground_cloud_out += ptCloud_nodewise_ground_;
                nonground_cloud_out += ptCloud_nodewise_nonground_;
                obstacle_cloud_out += ptCloud_nodewise_obstacle_;
                outlier_cloud_out += ptCloud_nodewise_outliers_;
            }
            }
            }

            return;
        };

        // functions for visualization

        geometry_msgs::PolygonStamped setPlanarModel (const TriGridNode<PointType>& node_in, const TriGridIdx& node_idx) {
            geometry_msgs::PolygonStamped polygon_out;
            polygon_out.header = msg_header_;
            geometry_msgs::Point32 corner_0, corner_1, corner_2;
            int r_i = node_idx.row;
            int c_i = node_idx.col;
            int s_i = node_idx.tri;
            if (node_in.node_type == GROUND){
                switch (s_i){
                    case 0:
                        //topx lowy & topx topy
                        corner_1.x = (r_i+1)*TGF_RESOLUTION_+tgf_min_x; corner_1.y = (c_i)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_1.z = (-node_in.normal(0,0)*corner_1.x - node_in.normal(1,0)*corner_1.y-node_in.d)/node_in.normal(2,0);

                        corner_2.x = (r_i+1)*TGF_RESOLUTION_+tgf_min_x; corner_2.y = (c_i+1)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_2.z = (-node_in.normal(0,0)*corner_2.x - node_in.normal(1,0)*corner_2.y-node_in.d)/node_in.normal(2,0);

                        corner_0.x = (r_i+0.5)*TGF_RESOLUTION_+tgf_min_x; corner_0.y = (c_i+0.5)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_0.z = (-node_in.normal(0,0)*corner_0.x - node_in.normal(1,0)*corner_0.y-node_in.d)/node_in.normal(2,0);
                        break;
                    case 1:
                        //topx topy & lowx topy
                        corner_1.x = (r_i+1)*TGF_RESOLUTION_+tgf_min_x; corner_1.y = (c_i+1)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_1.z = (-node_in.normal(0,0)*corner_1.x - node_in.normal(1,0)*corner_1.y-node_in.d)/node_in.normal(2,0);

                        corner_2.x = (r_i)*TGF_RESOLUTION_+tgf_min_x; corner_2.y = (c_i+1)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_2.z = (-node_in.normal(0,0)*corner_2.x - node_in.normal(1,0)*corner_2.y-node_in.d)/node_in.normal(2,0);

                        corner_0.x = (r_i+0.5)*TGF_RESOLUTION_+tgf_min_x; corner_0.y = (c_i+0.5)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_0.z = (-node_in.normal(0,0)*corner_0.x - node_in.normal(1,0)*corner_0.y-node_in.d)/node_in.normal(2,0);
                        break;
                    case 2:
                        //lowx topy & lowx lowy
                        corner_1.x = (r_i)*TGF_RESOLUTION_+tgf_min_x; corner_1.y = (c_i+1)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_1.z = (-node_in.normal(0,0)*corner_1.x - node_in.normal(1,0)*corner_1.y-node_in.d)/node_in.normal(2,0);

                        corner_2.x = (r_i)*TGF_RESOLUTION_+tgf_min_x; corner_2.y = c_i*TGF_RESOLUTION_+tgf_min_y; 
                        corner_2.z = (-node_in.normal(0,0)*corner_2.x - node_in.normal(1,0)*corner_2.y-node_in.d)/node_in.normal(2,0);

                        corner_0.x = (r_i+0.5)*TGF_RESOLUTION_+tgf_min_x; corner_0.y = (c_i+0.5)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_0.z = (-node_in.normal(0,0)*corner_0.x - node_in.normal(1,0)*corner_0.y-node_in.d)/node_in.normal(2,0);
                        break;
                    case 3:
                        //lowx lowy & topx lowy 
                        corner_1.x = (r_i)*TGF_RESOLUTION_+tgf_min_x; corner_1.y = (c_i)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_1.z = (-node_in.normal(0,0)*corner_1.x - node_in.normal(1,0)*corner_1.y-node_in.d)/node_in.normal(2,0);

                        corner_2.x = (r_i+1)*TGF_RESOLUTION_+tgf_min_x; corner_2.y = (c_i)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_2.z = (-node_in.normal(0,0)*corner_2.x - node_in.normal(1,0)*corner_2.y-node_in.d)/node_in.normal(2,0);

                        corner_0.x = (r_i+0.5)*TGF_RESOLUTION_+tgf_min_x; corner_0.y = (c_i+0.5)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_0.z = (-node_in.normal(0,0)*corner_0.x - node_in.normal(1,0)*corner_0.y-node_in.d)/node_in.normal(2,0);
                        break;
                    default:
                        break;
                }        
            } else {
                switch (s_i){
                    case 0:
                        //topx lowy & topx topy
                        corner_1.x = (r_i+1)*TGF_RESOLUTION_+tgf_min_x; corner_1.y = (c_i)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_1.z = -2.0;

                        corner_2.x = (r_i+1)*TGF_RESOLUTION_+tgf_min_x; corner_2.y = (c_i+1)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_2.z = -2.0;

                        corner_0.x = (r_i+0.5)*TGF_RESOLUTION_+tgf_min_x; corner_0.y = (c_i+0.5)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_0.z = -2.0;
                        break;
                    case 1:
                        //topx topy & lowx topy
                        corner_1.x = (r_i+1)*TGF_RESOLUTION_+tgf_min_x; corner_1.y = (c_i+1)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_1.z = -2.0;

                        corner_2.x = (r_i)*TGF_RESOLUTION_+tgf_min_x; corner_2.y = (c_i+1)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_2.z = -2.0;

                        corner_0.x = (r_i+0.5)*TGF_RESOLUTION_+tgf_min_x; corner_0.y = (c_i+0.5)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_0.z = -2.0;
                        break;
                    case 2:
                        //lowx topy & lowx lowy
                        corner_1.x = (r_i)*TGF_RESOLUTION_+tgf_min_x; corner_1.y = (c_i+1)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_1.z = -2.0;

                        corner_2.x = (r_i)*TGF_RESOLUTION_+tgf_min_x; corner_2.y = c_i*TGF_RESOLUTION_+tgf_min_y; 
                        corner_2.z = -2.0;

                        corner_0.x = (r_i+0.5)*TGF_RESOLUTION_+tgf_min_x; corner_0.y = (c_i+0.5)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_0.z = -2.0;
                        break;
                    case 3:
                        //lowx lowy & topx lowy 
                        corner_1.x = (r_i)*TGF_RESOLUTION_+tgf_min_x; corner_1.y = (c_i)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_1.z = -2.0;

                        corner_2.x = (r_i+1)*TGF_RESOLUTION_+tgf_min_x; corner_2.y = (c_i)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_2.z = -2.0;

                        corner_0.x = (r_i+0.5)*TGF_RESOLUTION_+tgf_min_x; corner_0.y = (c_i+0.5)*TGF_RESOLUTION_+tgf_min_y; 
                        corner_0.z = -2.0;
                        break;
                    default:
                        std::cout << "the error in sub-idx" << std::endl;
                        break;
                }
            }

            polygon_out.polygon.points.reserve(3); 
            polygon_out.polygon.points.push_back(corner_0);
            polygon_out.polygon.points.push_back(corner_2);
            polygon_out.polygon.points.push_back(corner_1);
            return polygon_out;
        }

        void publishTriGridFieldGraph() {
            viz_trigrid_polygons_.header = msg_header_;
            viz_trigrid_polygons_.polygons.clear();
            viz_trigrid_polygons_.likelihood.clear();

            // visualize the graph: nodes
            for (int r_i = 0; r_i < rows_; r_i++){
            for (int c_i = 0; c_i < cols_; c_i++){
            for (int s_i = 0; s_i < (int) trigrid_field_[r_i][c_i].size(); s_i++){
                TriGridIdx curr_idx = {r_i, c_i, s_i};
                if (trigrid_field_[r_i][c_i][s_i].node_type == GROUND) {
                    viz_trigrid_polygons_.polygons.push_back(setPlanarModel(trigrid_field_[r_i][c_i][s_i], curr_idx));
                    // if (trigrid_field_[r_i][c_i][s_i].normal[2] < 0.99) {
                    //     viz_trigrid_polygons_.likelihood.push_back(0.25);
                    // } else {
                        viz_trigrid_polygons_.likelihood.push_back(0.0);
                    // }
                    
                } else if (trigrid_field_[r_i][c_i][s_i].node_type == NONGROUND) {
                    // continue;
                    viz_trigrid_polygons_.polygons.push_back(setPlanarModel(trigrid_field_[r_i][c_i][s_i], curr_idx));
                    viz_trigrid_polygons_.likelihood.push_back(1.0);

                } else if (trigrid_field_[r_i][c_i][s_i].node_type == UNKNOWN) {
                    // continue;
                    if (trigrid_field_[r_i][c_i][s_i].is_curr_data) {
                        viz_trigrid_polygons_.polygons.push_back(setPlanarModel(trigrid_field_[r_i][c_i][s_i], curr_idx));
                        viz_trigrid_polygons_.likelihood.push_back(0.5);
                    }
                } else {
                    ROS_WARN("Unknown Node Type");
                }            
            }
            }
            }

            //visualize the graph: edges
            viz_trigrid_edges_.header = msg_header_;
            viz_trigrid_edges_.points.clear();
            geometry_msgs::Point src_pt;
            geometry_msgs::Point tgt_pt;
            for (int e_i = 0; e_i < (int) trigrid_edges_.size(); e_i++){
                if (trigrid_edges_[e_i].is_traversable){
                    viz_trigrid_edges_.color.a = 1.0;
                    viz_trigrid_edges_.color.r = 1.0;
                    viz_trigrid_edges_.color.g = 1.0;
                    viz_trigrid_edges_.color.b = 0.0;
                } else {
                    viz_trigrid_edges_.color.a = 0.1;
                    viz_trigrid_edges_.color.r = 1.0;
                    viz_trigrid_edges_.color.g = 1.0;
                    viz_trigrid_edges_.color.b = 1.0;
                }

                src_pt.x = trigrid_field_[trigrid_edges_[e_i].Pair.first.row][trigrid_edges_[e_i].Pair.first.col][trigrid_edges_[e_i].Pair.first.tri].mean_pt[0];
                src_pt.y = trigrid_field_[trigrid_edges_[e_i].Pair.first.row][trigrid_edges_[e_i].Pair.first.col][trigrid_edges_[e_i].Pair.first.tri].mean_pt[1];
                src_pt.z = trigrid_field_[trigrid_edges_[e_i].Pair.first.row][trigrid_edges_[e_i].Pair.first.col][trigrid_edges_[e_i].Pair.first.tri].mean_pt[2];

                tgt_pt.x = trigrid_field_[trigrid_edges_[e_i].Pair.second.row][trigrid_edges_[e_i].Pair.second.col][trigrid_edges_[e_i].Pair.second.tri].mean_pt[0];
                tgt_pt.y = trigrid_field_[trigrid_edges_[e_i].Pair.second.row][trigrid_edges_[e_i].Pair.second.col][trigrid_edges_[e_i].Pair.second.tri].mean_pt[1];
                tgt_pt.z = trigrid_field_[trigrid_edges_[e_i].Pair.second.row][trigrid_edges_[e_i].Pair.second.col][trigrid_edges_[e_i].Pair.second.tri].mean_pt[2];

                viz_trigrid_edges_.points.push_back(src_pt);
                viz_trigrid_edges_.points.push_back(tgt_pt);
            }

            // pub_trigrid_nodes_.publish(viz_trigrid_polygons_);
            // pub_trigrid_edges_.publish(viz_trigrid_edges_);
            return;
        };

        // void publishTriGridCorners() {
        //     viz_trigrid_corners_.header = cloud_header_;
        //     viz_trigrid_corners_.points.clear();

        //     TriGridCorner curr_corner;
        //     pcl::PointXYZ corner_pt;
        //     // for corners
        //     for (int r_i = 0; r_i < (int) trigrid_corners_.size(); r_i++){
        //     for (int c_i = 0; c_i < (int) trigrid_corners_[0].size(); c_i++){
        //         curr_corner = trigrid_corners_[r_i][c_i];
        //         if (curr_corner.zs.size() != curr_corner.weights.size()){
        //             ROS_WARN("ERROR in corners");
        //         }
        //         for (int i = 0; i < (int) curr_corner.zs.size(); i++){
        //             corner_pt.x = curr_corner.x;
        //             corner_pt.y = curr_corner.y;
        //             corner_pt.z = curr_corner.zs[i];
        //             viz_trigrid_corners_.points.push_back(corner_pt);
        //         }
        //     }
        //     }

        //     // for centers
        //     for (int r_i = 0; r_i < (int) trigrid_centers_.size(); r_i++){
        //     for (int c_i = 0; c_i < (int) trigrid_centers_[0].size(); c_i++){
        //         curr_corner = trigrid_centers_[r_i][c_i];
        //         if (curr_corner.zs.size() != curr_corner.weights.size()){
        //             ROS_WARN("ERROR in corners");
        //         }
        //         for (int i = 0; i < (int) curr_corner.zs.size(); i++){
        //             corner_pt.x = curr_corner.x;
        //             corner_pt.y = curr_corner.y;
        //             corner_pt.z = curr_corner.zs[i];
        //             viz_trigrid_corners_.points.push_back(corner_pt);
        //         }
        //     }
        //     }

        //     pub_trigrid_corners_.publish(viz_trigrid_corners_);
        //     return;
        // };

        // sensor_msgs::PointCloud2 convertCloudToRosMsg(pcl::PointCloud<PointType>& cloud, std::string &frame_id) {
        //     sensor_msgs::PointCloud2 cloud_msg;
        //     pcl::toROSMsg(cloud, cloud_msg);
        //     cloud_msg.header.frame_id = frame_id;
        //     return cloud_msg;
        // };

        // void publishPointClouds(){
        //     ptCloud_tgfwise_ground_.header = cloud_header_;
        //     pub_tgseg_ground_cloud.publish(convertCloudToRosMsg(ptCloud_tgfwise_ground_, cloud_header_.frame_id));
            
        //     ptCloud_tgfwise_nonground_.header = cloud_header_;
        //     pub_tgseg_nonground_cloud.publish(convertCloudToRosMsg(ptCloud_tgfwise_nonground_, cloud_header_.frame_id));

        //     ptCloud_tgfwise_outliers_.header = cloud_header_;
        //     pub_tgseg_outliers_cloud.publish(convertCloudToRosMsg(ptCloud_tgfwise_outliers_, cloud_header_.frame_id));
        // }
    };
}
#endif