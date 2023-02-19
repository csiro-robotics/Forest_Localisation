////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef TREE_HPP
#define TREE_HPP

#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include <pcl/common/common.h>

class Tree
{
    public: 
      Tree(){ };
      ~Tree(){};

      int id;
      Eigen::Vector3d position;
      double radius;
      double max_height;
      double min_height;
      pcl::PointXYZ min_point;
      int min_point_idx;

      pcl::PointCloud<pcl::PointXYZ> upper_cloud;
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::PointCloud<pcl::PointXYZ> crown_points;
      

};

#endif