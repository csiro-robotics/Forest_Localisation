////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _CONFIG_SERVER_H
#define _CONFIG_SERVER_H

#include <ros/ros.h>

#include <iostream>

class ConfigServer  
{
    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    std::stringstream ss_config_;

   

    public:

        ConfigServer()
        {

        }

        ConfigServer(ros::NodeHandle node, ros::NodeHandle private_node): node_(node), private_node_(private_node)
        {
            getRosParam<std::string>(private_node_,"root_folder", root_folder_, "");
            getRosParam<std::string>(private_node_,"save_folder",save_folder_, "~/Documents/");
            getRosParam<std::string>(private_node_,"dataset_name", dataset_name_, "");
   
            getRosParam<int>(private_node_,"traj_search_window", traj_search_window_, 10);    
            getRosParam<std::string>(private_node_,"groundtruth_trajectory_name", groundtruth_trajectory_name_, "");
            
            getRosParam<double>(private_node_,"ground_z_down_threshold", ground_z_down_threshold_, 1.3);
            getRosParam<double>(private_node_,"ground_z_up_threshold", ground_z_up_threshold_, 4.5);
            getRosParam<double>(private_node_,"tree_height", tree_height_, 3.0);
            getRosParam<double>(private_node_,"tolerance", tolerance_, 0.45);
            getRosParam<double>(private_node_,"min_cluster_size", min_cluster_size_, 50);
            getRosParam<double>(private_node_,"max_cluster_size", max_cluster_size_, 20000);
            getRosParam<double>(private_node_,"downsample_voxel_size", downsample_voxel_size_, 0.25);
            getRosParam<std::string>(private_node_,"base_frame", base_frame_, "base_link");
            getRosParam<std::string>(private_node_,"map_frame", map_frame_, "map");

            getRosParam<std::string>(private_node_,"imu_topic", imu_topic_, "imu");
            getRosParam<std::string>(private_node_,"gator_data_topic", gator_data_topic_, "gator_info");
            
            getRosParam<double>(private_node_,"distance_threshold_stopped", distance_thresh_, 0.01);
            getRosParam<double>(private_node_,"angle_threshold_stopped", angle_thresh_, 5*M_PI/180);  
            getRosParam<double>(private_node_,"velocity_threshold_stopped", avg_velocity_thresh_, 0.3);

            getRosParam<int>(private_node_,"number_particles", numParticles_, 1000);
      
            getRosParam<double>(private_node_,"odom_alpha1", alpha1_, 0.2);
            getRosParam<double>(private_node_,"odom_alpha2", alpha2_, 0.2);
            getRosParam<double>(private_node_,"odom_alpha3", alpha3_, 0.3);
            getRosParam<double>(private_node_,"odom_alpha4", alpha4_, 0.2);

            std::vector<double> init_pose = {0, 0, 0};
            getRosParam<double>(private_node_,"initial_pose", initial_pose_, init_pose);
      
            std::vector<float> empty;
            getRosParam<float>(private_node_, "trajectory_rotation", trajectory_rotation_, empty);
            getRosParam<float>(private_node_, "trajectory_translation", trajectory_translation_, empty);

            getRosParam<double>(private_node_,"laser_accumulate_time", laser_accum_time_, 1.5);
            getRosParam<bool>(private_node_,"compensate_cloud", compensate_cloud_, false);

            getRosParam<double>(private_node_,"particle_range_x", particle_range_x_, 1.0);
            getRosParam<double>(private_node_,"particle_range_y", particle_range_y_, 1.0);
     
            getRosParam<int>(private_node_,"min_landmarks_threshold",min_landmarks_threshold_, 2);

            getRosParam<std::string>(private_node_,"map_file_name",map_file_name_, "");
            getRosParam<double>(private_node_,"map_image_grid_size",grid_size_, 0.25);
            getRosParam<double>(private_node_,"map_image_max_height",map_max_height_, 34.28);      
            getRosParam<double>(private_node_,"map_image_frame_x",map_image_frame_x_, -228.84);
            getRosParam<double>(private_node_,"map_image_frame_y",map_image_frame_y_, 168.46);

            getRosParam<double>(private_node_,"ssd_beta",ssd_beta_, 1.0);
            getRosParam<double>(private_node_,"crown_radius",crown_radius_, 2.0);
            
            getRosParam<double>(private_node_,"wheels_axis_distance",wheels_axis_distance_, 1.96);
            
            getRosParam<int>(private_node_,"number_of_threads",num_threads_, 4);

            std::vector<int> map_display_window_default;
            getRosParam<int>(private_node_,"map_display_window",map_display_window_, map_display_window_default);

            std::vector<int> default_color = {255,0,0};
            getRosParam<int>(private_node_,"trunks_color",trunks_color_, default_color);

            getRosParam<bool>(private_node_,"using_lidarodom_model",using_lidarodom_model_, false);
            getRosParam<std::string>(private_node_,"lidarodom_model",lidarodom_model_, "");
            getRosParam<std::string>(private_node_,"input_lidar_topic",input_lidar_topic_, "cloud");
            
        }

        template <typename T> void getRosParam(ros::NodeHandle nh, std::string param_field, T& variable, T default_value)
        {
            nh.param<T>(param_field, variable, default_value);
            std::cout << param_field << ": " << variable <<"\n";
            ss_config_<<param_field << ": " << variable <<"\n";
        }
        
        template <typename T> void getRosParam(ros::NodeHandle nh, std::string param_field, std::vector<T>& variable, std::vector<T> default_value)
        {
            nh.param<std::vector<T>>(param_field, variable, default_value);
            
            std::cout << param_field << ": [";
            ss_config_<<param_field << ": [" ;
            for (const auto& item: variable)
            {
            ss_config_<<item<<" ";
            std::cout <<item<<" ";
            }
            ss_config_<<"]"<<"\n"; 
            std::cout <<"]"<<std::endl;
        }

        std::string getAllParamsString()
        {
            return ss_config_.str();
        }

       
        //Frames    
        std::string base_frame_;
        std::string map_frame_;
               
        //Subscribed and published topics
        std::string imu_topic_;
        std::string gator_data_topic_;
        std::string input_lidar_topic_;

        //Tree segmentation params
        double ground_z_down_threshold_;
        double ground_z_up_threshold_;
        double tree_height_;
        double tolerance_;
        double min_cluster_size_;
        double max_cluster_size_;
        double downsample_voxel_size_;

        double distance_thresh_;
        double angle_thresh_;
        double avg_velocity_thresh_;

        int numParticles_;
      
        double alpha1_;
        double alpha2_;
        double alpha3_;
        double alpha4_;
        
        double wheels_axis_distance_;

        std::vector<double> initial_pose_;
        
        std::vector<float> trajectory_rotation_;
        std::vector<float> trajectory_translation_;

        std::string root_folder_;
        std::string dataset_name_;
        std::string groundtruth_trajectory_name_;
        int traj_search_window_;

        bool compensate_cloud_;
        double laser_accum_time_;
        double particle_range_x_;
        double particle_range_y_;
     
        std::string save_folder_;
    
        int min_landmarks_threshold_;

        //ACM map param
        std::string map_file_name_;
        double grid_size_;
        double map_max_height_;
        double map_image_frame_x_;
        double map_image_frame_y_;

        double ssd_beta_;
        double crown_radius_;

        int num_threads_;
        std::vector<int> map_display_window_;
        std::vector<int> trunks_color_;

        bool using_lidarodom_model_;
        std::string lidarodom_model_;
     

};  
#endif