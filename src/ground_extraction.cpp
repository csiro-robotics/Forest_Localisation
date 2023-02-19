////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ground_extraction.h"


GroundExtraction::GroundExtraction(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  //Code partially extracted from the linefit_ground_segmentation package (https://github.com/lorenwel/linefit_ground_segmentation)
  nh_private.param<std::string>("ground_output_topic", ground_topic_, "");
  nh_private.param<std::string>("obstacle_output_topic", obstacle_topic_, "");
  nh_private.param("latch", latch_, false);

  ground_pub_ = nh_private.advertise<pcl::PointCloud<pcl::PointXYZ>>(ground_topic_, 1, latch_);
  obstacle_pub_ = nh_private.advertise<pcl::PointCloud<pcl::PointXYZ>>(obstacle_topic_, 1, latch_);

  //linefit_ground_segmentation params
  nh_private.param("visualize", params_.visualize, params_.visualize);
  nh_private.param("n_bins", params_.n_bins, params_.n_bins);
  nh_private.param("n_segments", params_.n_segments, params_.n_segments);
  nh_private.param("max_dist_to_line", params_.max_dist_to_line, params_.max_dist_to_line);
  nh_private.param("max_slope", params_.max_slope, params_.max_slope);
  nh_private.param("min_slope", params_.min_slope, params_.min_slope);
  nh_private.param("long_threshold", params_.long_threshold, params_.long_threshold);
  nh_private.param("max_long_height", params_.max_long_height, params_.max_long_height);
  nh_private.param("max_start_height", params_.max_start_height, params_.max_start_height);
  nh_private.param("sensor_height", params_.sensor_height, params_.sensor_height);
  nh_private.param("line_search_angle", params_.line_search_angle, params_.line_search_angle);
  nh_private.param("n_threads", params_.n_threads, params_.n_threads);
  
  double r_min, r_max, max_fit_error;
  if (nh_private.getParam("r_min", r_min)) {
    params_.r_min_square = r_min*r_min;
  }
  if (nh_private.getParam("r_max", r_max)) {
    params_.r_max_square = r_max*r_max;
  }
  if (nh_private.getParam("max_fit_error", max_fit_error)) {
    params_.max_error_square = max_fit_error * max_fit_error;
  }
  
}

void GroundExtraction::extractGround(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    
    if(ground_cloud_.points.size()>0)
      ground_cloud_.points.clear();
    if(no_ground_cloud_.points.size()>0)  
     no_ground_cloud_.clear();	
      
    GroundSegmentation segmenter(params_);
    pcl::PointCloud<pcl::PointXYZ> cloud_transformed;

    std::vector<int> labels;

    const pcl::PointCloud<pcl::PointXYZ>& cloud_proc =  cloud;

    segmenter.segment(cloud, &labels);

    ground_cloud_.header = cloud.header;
    no_ground_cloud_.header = cloud.header;

    for (size_t i = 0; i < cloud.size(); ++i) {
    	if (labels[i] == 1) ground_cloud_.push_back(cloud[i]);      
	    else no_ground_cloud_.push_back(cloud[i]);
    }

    std::cout<<"Number of ground points: "<<ground_cloud_.points.size()<<endl;
    std::cout<<"Number of Non ground points: "<<no_ground_cloud_.points.size()<<endl;

}


void GroundExtraction::splitCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &cloud_out, double z_min, double z_max)
{

  if(cloud_ground_grid_.points.size()==0)
  {
    ROS_WARN("Ground not detected: returning same pointcloud from split function");
    pcl::copyPointCloud(cloud, cloud_out);
    return;
  }
  
  cloud_out.points.reserve(cloud.points.size());


  for(int k=0;k<cloud.points.size(); k++)
  {
    pcl::PointXYZ point = cloud.points[k];
    
    int i = int((point.y -  local_frame_[1])/grid_size_);
    int j = int((point.x - local_frame_[0])/grid_size_);
    bool has_ground = false;

    //Check if has ground point to compare the heights
    if(i>=0 && j>=0 && i<height_rows_ && j<height_cols_)
    {
      if(cloud_ground_grid_.at(j,i).z != 1e10)
      {
        double height_diff = point.z - cloud_ground_grid_.at(j,i).z;
        has_ground = true;
	      if(height_diff> z_min && height_diff < z_max)
          cloud_out.points.push_back(point);        
      }      
    }

    //If there is no ground point to compare get the cloud point anyway 
    if(!has_ground)
      cloud_out.points.push_back(point);

  }
  
  cloud_out.height = 1;
  cloud_out.width = cloud_out.points.size();

}



void GroundExtraction::getLowGroundCloud()
{
  //Get a grid of points (extracted from the ground segmentation) with lower heights  
  
  if(ground_cloud_.points.size() < 10)
  {
    ROS_WARN("Ground points returned less than 10 points. Low cloud cannot be computed");
    cloud_ground_grid_.clear();
    return;
  }
  
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(ground_cloud_,minPt,maxPt);

  //Bottom left point used as reference to index the grid (cloud_ground_grid_)
  local_frame_[0] = minPt.x;
  local_frame_[1] = minPt.y;
  
  double grid_max_x_ = maxPt.x;
  double grid_min_x_ = minPt.x;
  double grid_max_y_ = maxPt.y;
  double grid_min_y_ = minPt.y;

  grid_size_ = 2.0;
  height_rows_ = (int)ceil((grid_max_y_-grid_min_y_)/grid_size_);
  height_cols_ = (int)ceil((grid_max_x_-grid_min_x_)/grid_size_);
  
  if(height_rows_<2 || height_cols_<2)
  {
    ROS_WARN("Ground points returned grid has dimensions lower than 2: Low cloud not conputed");
    cloud_ground_grid_.clear();
    return;
  }

  cloud_ground_grid_ = pcl::PointCloud<pcl::PointXYZ>(height_cols_,height_rows_,pcl::PointXYZ(0,0,1e10));

  for(auto &point: ground_cloud_.points)
  {
      
      int i = int((point.y -  local_frame_[1])/grid_size_);
      int j = int((point.x - local_frame_[0])/grid_size_);

      if(i<0 || j<0 || i>=height_rows_ || j>=height_cols_)
      {
        printf("i , j (%i, %i) \n", i, j);
        printf("rows , cols (%i, %i) \n", height_rows_, height_cols_);
        printf("corners minx %0.2f, maxx %0.2f, miny %0.2f, maxy %0.2f \n", grid_min_x_,grid_max_x_, grid_min_y_, grid_max_y_);
       // printf("highcloud minx %0.2f, maxx %0.2f, miny %0.2f, maxy %0.2f \n", min_x,max_x, min_y, max_y);
      }
      
      cloud_ground_grid_.at(j,i).z = std::min(cloud_ground_grid_.at(j,i).z,point.z);
  }

  for(int i=0; i<height_rows_; i++)
  {
    for(int j=0; j<height_cols_; j++)
    {
      double x = j*grid_size_ + local_frame_[0];
      double y = i*grid_size_ + local_frame_[1];
      cloud_ground_grid_.at(j,i).x = x;
      cloud_ground_grid_.at(j,i).y = y;      
    }
  }  

  
}





      
