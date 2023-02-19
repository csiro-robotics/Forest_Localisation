////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _PFILTER_LOCALIZATION_H
#define _PFILTER_LOCALIZATION_H

#include "map.h"
#include "utils.h"
#include "config_server.h"
#include "tree_segmentation.h"
#include "csv.h"
#include "visualisation.h"
#include "imu_interface.h"

#include <cmath>  
#include <iostream>
#include <random>

#include "ros/ros.h"

#include "forest_localisation/gator_data.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/TransformStamped.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <thread>
#include <cv_bridge/cv_bridge.h>

using namespace std;

struct particle_state  
{   
    particle_state(){
    }

    particle_state(float x_in, float y_in, float theta_in, float weight_in): x(x_in), y(y_in), theta(theta_in), weight(weight_in){
    }

	float x;
	float y;
	float theta;
	float weight;
};


struct CarLikeModel{
    CarLikeModel(double distance_wheels_in = 0): distance_wheels(distance_wheels_in)
    {
        steering_pos=0;
        time = 0;
    }
    
    double steering_pos;
    double distance_wheels;
    double dx,dy,dtheta, v_linear;
    double time;

    
    void computeVelocities(double theta)
    {
        dtheta = tan(steering_pos)*v_linear/distance_wheels;
        dx = cos(theta)*v_linear;
        dy = sin(theta)*v_linear;     
    };

    void updateOdometryPose(pose2D_t &odom_pose, double current_time)
    {
        double dt = current_time - time;
        odom_pose.x = odom_pose.x + dt*dx;
        odom_pose.y = odom_pose.y + dt*dy;
        odom_pose.theta = odom_pose.theta + dt*dtheta;

        time = current_time;
    }

};


class PfilterLocalization
{

public:
    PfilterLocalization(ros::NodeHandle node, ros::NodeHandle private_node);
    ~PfilterLocalization();
    void InitParticles(pose2D_t initial_pose);
	void PfilterAlgorithm();
    
    void laserCloudReceived(const sensor_msgs::PointCloud2ConstPtr& laser_scan);
    void gatorInfoWheelOdom(const forest_localisation::gator_dataConstPtr &gator_msg);
    bool accumulateLidarScansWheel(const sensor_msgs::PointCloud2ConstPtr& laser_scan);
    void updateGroundTruthPose();
    void extractTreeTrunks();

    void setInitialPoses();
  
    void mapMatchingThread(int init_id, int end_id);
    double mapMatchingParticle(pose2D_t pose_particle, int particle_id);
    void similarityMeasure(cv::Mat &local_image, cv::Mat &map_image, double &score, cv::Point particle_pos_pxl, cv::Point particle_pos_local_pxl, Eigen::Vector2d local_frame);
  
    void publishMsgs();
    bool saveData();
  
    void publishParticles(vector<particle_state> particles, vector<particle_state> resampled_particles);
    void maximumParticleWeights(particle_state & highest_w_particle, int &particle_id, vector<double> particle_weights);
   
    std::shared_ptr<ConfigServer> config_;
 
private:
    particle_state sampleCarMotionModel(particle_state particle);

    double sampleNormalDistribution(double std_dev);
    void LowVarianceSampler();
    double ProbNormal(double x, double std_dev);
    void preparePoseMsg(pose2D_t pose, geometry_msgs::PoseStamped &msg, string frame, ros::Time stamp);
    void preparePoseMsg(pose2D_t pose, geometry_msgs::Pose &msg, string frame, ros::Time stamp);

    ros::NodeHandle node_;
    ros::NodeHandle private_node_; 
    std::shared_ptr<Visualisation> visualisation_;
    std::unique_ptr<ImuInterface> imu_interface_;
    std::unique_ptr<TreeSegmentation> tree_segmentation_;

    std::shared_ptr<tf2_ros::Buffer> tf_;
  
    TrajectoryData groundtruth_trajectory_;     
    tf2::Transform groundtruth_current_transform_;
    pose2D_t groundtruth_pose_;

    ros::Subscriber laser_cloud_sub_;
    ros::Subscriber gator_info_sub_;  
   
    std::random_device random_dv{};
    std::mt19937 random_gen;
    
    std::string base_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    ros::Publisher publish_particles_;
    ros::Publisher publish_pose_;
    ros::Publisher publish_gt_pose_;
    ros::Publisher publish_odom_pose_;
    
    ros::Publisher publish_particles_update_;
    ros::Publisher publish_particles_resample_;
    geometry_msgs::PoseArray particles_update_msg_;
    geometry_msgs::PoseArray particles_resample_msg_;

    ros::Publisher publish_pose_path_;
    ros::Publisher publish_groundtruth_path_;
    ros::Publisher publish_odom_pose_path_;

    ros::Publisher publish_max_weight_poses_;
    
    geometry_msgs::PoseStamped robot_pos_msg_;
    geometry_msgs::PoseStamped odom_pose_msg_;
   
    nav_msgs::Path robot_pos_path_;
    nav_msgs::Path robot_odom_path_;
    nav_msgs::Path groundtruth_path_;
    
    vector<Eigen::Vector2i> robot_image_pos_;

    ros::Publisher scan_pub_;
   
    vector<particle_state> particles_;
    std::vector<double> new_weights_;
    double total_weight_;
    pose2D_t robot_pose_;
    
    bool robot_moved_;

    ImageMap image_map_;    
   
    vector<cv::Mat> ucm_images_buffer_;
    vector<Eigen::Vector2d> local_frames_buffer_;

    pose2D_t last_odom_pose_;
    pose2D_t odom_pose_;
      
    pcl::PointCloud<pcl::PointXYZ> sweep_cloud_pcl_;
    pcl::PointCloud<pcl::PointXYZ> accumulated_cloud_pcl_;

    double current_sweep_time_;
    double start_sweep_time_;
    ros::Time current_laser_stamp_;
   
    bool first_scan; 
    bool set_start_position_;
    
    CarLikeModel car_model;
  
    bool first_gator_fb;
  
    std::vector<std::thread> pfilter_threads_; 
    std::mutex data_assoc_mutex_;

   
};

#endif



