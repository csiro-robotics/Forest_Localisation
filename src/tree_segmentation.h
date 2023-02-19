////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _TREE_SEGMENTATION_H
#define _TREE_SEGMENTATION_H

#include "ros/ros.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"

#include <string.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "tree.h"
#include "ground_extraction.h"
#include "config_server.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

class TreeSegmentation {   
    
  public: 
     TreeSegmentation(ros::NodeHandle node, ros::NodeHandle private_node, std::shared_ptr<ConfigServer>& config_ptr);
    ~TreeSegmentation(); 
    void downSizeCloud(PointCloudXYZ &cloud, PointCloudXYZ &filtered_cloud);
    bool extractTrunks(PointCloudXYZ &cloud);
    void splitCloudByHeights(PointCloudXYZ &cloud, PointCloudXYZ &cloud_cropped);
    void euclideanSegmentation(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    void findMaximumTreesHeights(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    Eigen::Vector3d getCloudCentroid(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    void getClusterHeight(Tree& tree);

    void generateUCM(std::vector<Tree> extracted_trunks_transformed, Eigen::Vector2d &local_frame, cv::Mat &ucm_image, double map_max_height);


    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    std::shared_ptr<ConfigServer> config_;
    double voxel_size_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster_;
    std::unique_ptr<GroundExtraction> ground_extraction_;
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud_;

    std::vector<Tree> extracted_trunks_;
};

#endif