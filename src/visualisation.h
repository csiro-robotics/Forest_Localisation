////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef VISUALISATION_HPP
#define VISUALISATION_HPP

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Path.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include "tree.h"
#include "config_server.h"
#include "utils.h"
#include "map.h"


struct RGB_color{
    float r;
    float g;
    float b;
};

class Visualisation
{


public:
    Visualisation(ros::NodeHandle node, ros::NodeHandle private_node, std::shared_ptr<ConfigServer>& config_ptr);
    ~Visualisation();

    void publishClustersSegments(std::vector<Tree> trees,  ros::Time timestamp);
    void publishCloudInput(pcl::PointCloud<pcl::PointXYZ>&pcloud, ros::Time timestamp);
    void publishDownsampledSweep(pcl::PointCloud<pcl::PointXYZ>&cloud, ros::Time timestamp);
    void publishTrunksExtracted(std::vector<Tree> trees, ros::Time timestamp);
    void publishGroundCloud(pcl::PointCloud<pcl::PointXYZ>&pcloud, ros::Time timestamp);
    void colorCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, RGB_color colour);
    void publishMapAndBeliefImages(pose2D_t pose_particle, Eigen::Vector2d local_frame, cv::Mat ucm_image, ImageMap acm_map, const std::vector<Eigen::Vector2i> &robot_poses_pixel);
    void lineDot(cv::OutputArray img, const cv::Point& pt1, const cv::Point& pt2, const cv::Scalar& color, const vector<bool>& pattern);

    //void publishTrackedTrees(std::vector<Trunk> tracked_trees, ros::Time timestamp);
    std::vector<RGB_color> rgb_rand_color( int n_colors);
   
private:
    


    ros::NodeHandle node_;
    ros::NodeHandle private_node_; //for ros parameters
    std::shared_ptr<ConfigServer> config_;
 
     //Trunks markers in base link frame    
    ros::Publisher tree_pos_pub_;
    visualization_msgs::MarkerArray tree_pos_msg_;
    
    //Publish clustered tree points
    ros::Publisher clusters_segment_pub_;
    visualization_msgs::Marker clusters_segment_msg_;

    ros::Publisher publish_matched_images_;

    ros::Publisher sweep_cloud_;
    ros::Publisher filtered_sweep_pub_;
     ros::Publisher ground_cloud_pub_;
    
   


};


#endif