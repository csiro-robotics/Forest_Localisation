////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "visualisation.h"


Visualisation::Visualisation(ros::NodeHandle node, ros::NodeHandle private_node, std::shared_ptr<ConfigServer>& config_ptr)
{
    node_ = node;
    private_node_ = private_node;
    config_ = config_ptr;

    clusters_segment_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/trees_clusters_segments", 1,true);
    sweep_cloud_ = node_.advertise<sensor_msgs::PointCloud2>("/sweep_cloud",1, true);
    tree_pos_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/trees_positions_markers",1, true);
    filtered_sweep_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/downsampled_sweep",1, true);
    // ground_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/ground_grid",1, true);
    
    publish_matched_images_ = node_.advertise<sensor_msgs::Image>("/map_local_images",1);

}

Visualisation::~Visualisation()
{

}

void Visualisation::publishClustersSegments(std::vector<Tree> trees,  ros::Time timestamp)
{
    int n_clusters = trees.size();

    sensor_msgs::PointCloud2 clusters_msg;
    pcl::PointCloud<pcl::PointXYZRGB> clusters_clouds;
    
    int i = 0;
    
    //Creates n random colours
    std::vector<RGB_color> colors_rgb_vec;
    colors_rgb_vec = rgb_rand_color(n_clusters);

    for( auto &tree_obj: trees)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud_colour;
        pcl::copyPointCloud<pcl::PointXYZ,pcl::PointXYZRGB>(tree_obj.cloud, cloud_colour);
        colorCloud(cloud_colour,colors_rgb_vec[i]);
        i++;
        clusters_clouds = clusters_clouds + cloud_colour;
    }
    
    pcl::toROSMsg(clusters_clouds, clusters_msg);
    clusters_msg.header.frame_id = config_->base_frame_;
    clusters_msg.header.stamp = timestamp;

    clusters_segment_pub_.publish(clusters_msg);

  
}

void Visualisation::colorCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, RGB_color colour)
{
    int n_points = cloud.size();
    for(int i=0; i< n_points; i++)
    {
        cloud[i].r = uint8_t(colour.r*255);
        cloud[i].g = uint8_t(colour.g*255);;
        cloud[i].b = uint8_t(colour.b*255);;        
    }
}

std::vector<RGB_color> Visualisation::rgb_rand_color( int n_colors)
{
    std::vector<RGB_color> colours_vec;
    
    if(n_colors<=0)
    {
      std::cout<<"No cluster found"<<std::endl;
      return colours_vec;
    }

    for( int i=0; i<n_colors ; i++)
    {
      
      RGB_color colour_item;
      colour_item.r = float(rand()%256)/255.0;
      colour_item.g = float(rand()%256)/255.0;
      colour_item.b = float(rand()%256)/255.0;       

      colours_vec.push_back(colour_item);
    }
    return colours_vec;
}

void Visualisation::publishCloudInput(pcl::PointCloud<pcl::PointXYZ>&pcloud, ros::Time timestamp)
{
  sensor_msgs::PointCloud2 pcloud_msg;
  pcl::toROSMsg(pcloud,pcloud_msg);
  pcloud_msg.header.frame_id = config_->base_frame_;
  pcloud_msg.header.stamp = timestamp;
  sweep_cloud_.publish(pcloud_msg);
   
}

void Visualisation::publishDownsampledSweep(pcl::PointCloud<pcl::PointXYZ>&cloud, ros::Time timestamp)
{
  sensor_msgs::PointCloud2 pcloud_msg;
  pcl::toROSMsg(cloud,pcloud_msg);
  pcloud_msg.header.frame_id = config_->base_frame_;
  pcloud_msg.header.stamp = timestamp;
  filtered_sweep_pub_.publish(pcloud_msg);
   
}

void Visualisation::publishTrunksExtracted(std::vector<Tree> trees, ros::Time timestamp)
{

   if(tree_pos_msg_.markers.size()>0)
  {
    for(int k=0; k<tree_pos_msg_.markers.size(); k++)
    {
      tree_pos_msg_.markers[k].action = visualization_msgs::Marker::DELETE;
    }
    tree_pos_pub_.publish(tree_pos_msg_);
  }
  
  tree_pos_msg_.markers.clear();
  tree_pos_msg_.markers.resize(trees.size());

  ros::Time now_time = ros::Time::now();
  
  int j = 0;
  for(const auto& trunk: trees) 
  {
    
    tree_pos_msg_.markers[j].id = j;
    tree_pos_msg_.markers[j].header.stamp = now_time;
    tree_pos_msg_.markers[j].header.frame_id = config_->base_frame_;
    tree_pos_msg_.markers[j].ns ="trunks_extracted";
    tree_pos_msg_.markers[j].type = visualization_msgs::Marker::CYLINDER;
    tree_pos_msg_.markers[j].action = visualization_msgs::Marker::ADD;
    tree_pos_msg_.markers[j].pose.position.x = trunk.position[0];
    tree_pos_msg_.markers[j].pose.position.y = trunk.position[1];
    // double height = (trunk.max_height>0)? trunk.max_height: 1.0;
    tree_pos_msg_.markers[j].pose.position.z = (trunk.max_height>0) ? (trunk.position(2) + trunk.max_height/2) : trunk.position(2);
    tree_pos_msg_.markers[j].pose.orientation.x = 0.0;
    tree_pos_msg_.markers[j].pose.orientation.y = 0.0;
    tree_pos_msg_.markers[j].pose.orientation.z = 0.0;
    tree_pos_msg_.markers[j].pose.orientation.w = 1.0;
    tree_pos_msg_.markers[j].scale.x = 0.2;
    tree_pos_msg_.markers[j].scale.y = 0.2;
    tree_pos_msg_.markers[j].scale.z =  (trunk.max_height>0) ? trunk.max_height : 1.0; //height;

    tree_pos_msg_.markers[j].color.r = 255;
    tree_pos_msg_.markers[j].color.g = 255;
    tree_pos_msg_.markers[j].color.b = 0;

    
    tree_pos_msg_.markers[j].color.a = 1.0;

    tree_pos_msg_.markers[j].lifetime = ros::Duration();
    j+=1;
  }
  tree_pos_pub_.publish(tree_pos_msg_);
   
}


void Visualisation::publishGroundCloud(pcl::PointCloud<pcl::PointXYZ>&pcloud, ros::Time timestamp)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud_ground_color(pcloud.points.size(),1,pcl::PointXYZRGB(255,0,0));
    pcl::copyPointCloud(pcloud,cloud_ground_color);
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_ground_color,cloud_msg);
    cloud_msg.header.frame_id = "base_link";
    ground_cloud_pub_.publish(cloud_msg);
}

void Visualisation::publishMapAndBeliefImages(pose2D_t pose_particle, Eigen::Vector2d local_frame, cv::Mat ucm_image, ImageMap acm_map, const std::vector<Eigen::Vector2i> &robot_poses_pixel)
{

  double grid_size = acm_map.grid_size_;
  cv::Mat map_image_brg;
  cvtColor(acm_map.image_.clone(),map_image_brg,cv::COLOR_GRAY2BGR);

  cv::Mat ucm_image_color;
  
  // Apply the colormap:
  applyColorMap(ucm_image, ucm_image_color, cv::COLORMAP_AUTUMN);
  
  cv::Point particle_pos_pxl;
  particle_pos_pxl.x = (pose_particle.x - acm_map.image_frame_x_)/grid_size;
  particle_pos_pxl.y = (acm_map.image_frame_y_ - pose_particle.y)/grid_size;

  cv::Point particle_pos_local_pxl;
  particle_pos_local_pxl.x = (pose_particle.x - local_frame[0])/grid_size;
  particle_pos_local_pxl.y = (local_frame[1] - pose_particle.y)/grid_size;

  cv::Point template_corner;
  template_corner.x = std::max(particle_pos_pxl.x - particle_pos_local_pxl.x, 0);
  template_corner.y = std::max(particle_pos_pxl.y - particle_pos_local_pxl.y, 0);

  int diff_x = (particle_pos_pxl.x - particle_pos_local_pxl.x >= 0)? 0: (particle_pos_pxl.x - particle_pos_local_pxl.x);
  int diff_y = (particle_pos_pxl.y - particle_pos_local_pxl.y >= 0)? 0: (particle_pos_pxl.y - particle_pos_local_pxl.y);
  int dx = ucm_image.cols - diff_x;
  int dy = ucm_image.rows - diff_y;
 
  float cost = 0;
  float max_value_map = 0;


  cv::Mat_<cv::Vec3b> map_image_brg_temp = map_image_brg;
  cv::Mat_<cv::Vec3b> ucm_image_color_temp = ucm_image_color;
  for (int i = 0; i< dy ; i++)
  {
    for (int j=0; j< dx ; j++)
    {
               
      if(ucm_image.at<uchar>(i,j)>0)
      {
        map_image_brg_temp(template_corner.y + i,template_corner.x + j)[0] = ucm_image_color_temp(i,j)[0];
        map_image_brg_temp(template_corner.y + i,template_corner.x + j)[1] = ucm_image_color_temp(i,j)[1];
        map_image_brg_temp(template_corner.y + i,template_corner.x + j)[2] = ucm_image_color_temp(i,j)[2];           
      }        
     
    }

  }

  std::vector<bool> pattern = {0,0,0,1,1,1};
  lineDot(map_image_brg_temp,template_corner, cv::Point(template_corner.x + dx, template_corner.y),cv::Scalar(0,128,255), pattern);
  lineDot(map_image_brg_temp,cv::Point(template_corner.x + dx, template_corner.y), cv::Point(template_corner.x + dx, template_corner.y + dy),cv::Scalar(0,128,255), pattern);
  lineDot(map_image_brg_temp,cv::Point(template_corner.x + dx, template_corner.y + dy), cv::Point(template_corner.x, template_corner.y + dy),cv::Scalar(0,128,255), pattern);
  lineDot(map_image_brg_temp,cv::Point(template_corner.x, template_corner.y + dy), template_corner,cv::Scalar(0,128,255), pattern);
  

  //Draw robot estimated position    
  for(auto & robot_image_pos: robot_poses_pixel)   
  {  
    cv::Point r_img_pos;
    r_img_pos.x = robot_image_pos[0];
    r_img_pos.y = robot_image_pos[1];
    cv::circle(map_image_brg_temp, r_img_pos,2, cv::Scalar(0,255,255),cv::FILLED); //Scalar(0,255,128)
  }

  map_image_brg = map_image_brg_temp;
  
  if(config_->map_display_window_.size()>0)
  {
    if(config_->map_display_window_[0]<map_image_brg.cols && config_->map_display_window_[2]<map_image_brg.cols && config_->map_display_window_[1]<map_image_brg.rows && config_->map_display_window_[3]<map_image_brg.rows)
      map_image_brg = map_image_brg(cv::Range(config_->map_display_window_[1],config_->map_display_window_[3]),cv::Range(config_->map_display_window_[0],config_->map_display_window_[2]));
  }

  sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", map_image_brg).toImageMsg();

  publish_matched_images_.publish(image_msg);
}

void Visualisation::lineDot(cv::OutputArray img, const cv::Point& pt1, const cv::Point& pt2, const cv::Scalar& color, const vector<bool>& pattern)
{
    cv::LineIterator it(img.getMat(), pt1, pt2, cv::LINE_8); 
    for(auto i=0; i<it.count; i++, it++){
        if(pattern[i%pattern.size()]){ // use any pattern of any length, dotted is {0,0,1}, dashed is {0,0,0,1,1,1} etc
            (*it)[0] = color.val[0];
            (*it)[1] = color.val[1];
            (*it)[2] = color.val[2];
        }
    }
}