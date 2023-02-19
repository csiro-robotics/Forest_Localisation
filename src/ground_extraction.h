////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _GROUND_EXTRACTION_H
#define _GROUND_EXTRACTION_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>

#include "ground_segmentation/ground_segmentation.h"

//Interface class to utilise the linefit_ground_segmentation package
class GroundExtraction {

  ros::Publisher ground_pub_;
  ros::Publisher obstacle_pub_;
  GroundSegmentationParams params_;

public:
  GroundExtraction(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  void extractGround(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  void getLowGroundCloud();
  void splitCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &cloud_out, double z_min, double z_max);
  

  std::string ground_topic_, obstacle_topic_;
  bool latch_;
  pcl::PointCloud<pcl::PointXYZ> ground_cloud_;
  pcl::PointCloud<pcl::PointXYZ> no_ground_cloud_;
  
  //Ground grid parameters
  pcl::PointCloud<pcl::PointXYZ> cloud_ground_grid_;
  Eigen::Vector2d local_frame_;
  int height_rows_;
  int height_cols_;
  double grid_size_;

};


#endif
