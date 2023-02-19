////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "tree_segmentation.h"

TreeSegmentation::TreeSegmentation(ros::NodeHandle node, ros::NodeHandle private_node, std::shared_ptr<ConfigServer>& config_ptr)
{
    node_ = node;
    private_node_ = private_node;
    config_ = config_ptr;
    ground_extraction_.reset(new GroundExtraction(node_, private_node_));
    kdtree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);

    euclidean_cluster_.setClusterTolerance(config_->tolerance_);
    euclidean_cluster_.setMinClusterSize(config_->min_cluster_size_);
    euclidean_cluster_.setMaxClusterSize(config_->max_cluster_size_);
    euclidean_cluster_.setSearchMethod(kdtree_);
    voxel_size_ = config_->downsample_voxel_size_;
}

TreeSegmentation::~TreeSegmentation()
{

}

void TreeSegmentation::downSizeCloud(PointCloudXYZ &cloud, PointCloudXYZ &filtered_cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	*cloud_ptr = cloud;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (cloud_ptr);
	vg.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
	vg.filter (filtered_cloud);
	ROS_INFO("PointCloud after filtering (voxelisation) has: %d data points." , filtered_cloud.size ());
}

bool TreeSegmentation::extractTrunks(PointCloudXYZ &cloud)
{
  ROS_INFO("Extracting trunks");
  
  pcl::PointCloud<pcl::PointXYZ> cloud_cropped;

  // std::cout << "Received " << cloud.width * cloud.height << " points" << std::endl;
  extracted_trunks_.clear();

  downSizeCloud(cloud, filtered_cloud_);  
  splitCloudByHeights(filtered_cloud_, cloud_cropped);
  euclideanSegmentation(cloud_cropped);

  int n_trees = extracted_trunks_.size();
  ROS_INFO("Number of trees found: %d",n_trees);
  
   if(n_trees<=0)
    return false;

  findMaximumTreesHeights(cloud);

  return true;

}

void TreeSegmentation::splitCloudByHeights(PointCloudXYZ &cloud, PointCloudXYZ &cloud_cropped)
{
  //Get points with heights between min and max thresholds

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor_out(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

  ground_extraction_->extractGround(cloud);
  ground_extraction_->getLowGroundCloud();
  ground_extraction_->splitCloud(cloud,cloud_cropped, config_->ground_z_down_threshold_, config_->ground_z_up_threshold_);
}


void TreeSegmentation::euclideanSegmentation(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  *cloud_ptr = cloud;
  std::vector<pcl::PointIndices> cluster_indices;

  kdtree_->setInputCloud (cloud_ptr);
  euclidean_cluster_.setInputCloud(cloud_ptr);
  euclidean_cluster_.extract(cluster_indices);
  std::cout<<"Euclidean clustering..."<<std::endl;
  std::cout << "Euclidean clustering size (number of clusters): " << cluster_indices.size() << std::endl;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) 
  {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) 
    {
			pcl::PointXYZ point;
			point.x = cloud_ptr->points[*pit].x;
			point.y = cloud_ptr->points[*pit].y;
			point.z = cloud_ptr->points[*pit].z;

			cloud_cluster->push_back(point);

		}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		Tree tree_object;

		tree_object.cloud = *cloud_cluster;
		tree_object.position = getCloudCentroid(*cloud_cluster);
		extracted_trunks_.push_back(tree_object);
	}

  //Verify if clusters are trees by cheking the height of clusters
  for(int i=0; i<extracted_trunks_.size(); i++ )
  {
    getClusterHeight(extracted_trunks_[i]);

    bool is_tree = (extracted_trunks_[i].max_height - extracted_trunks_[i].min_height ) > config_->tree_height_;
    if(is_tree)
      continue;    
    else
    {
    // short clusters are removed from the list
      extracted_trunks_.erase(extracted_trunks_.begin()+i);
      i--;
    }
  }




}

Eigen::Vector3d TreeSegmentation::getCloudCentroid(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  Eigen::Vector3d centroid;
	int cloud_size = cloud.points.size();
	float x = 0; 
	float y = 0; 
	float z = 0;
	
	for(auto &point: cloud.points)
  {
      x +=  point.x;
			y +=  point.y;
			z +=  point.z;
  }

	centroid[0] = x / cloud_size;
	centroid[1] = y / cloud_size;
	centroid[2] = z / cloud_size;

	return centroid;
}

void TreeSegmentation::getClusterHeight(Tree& tree)
{
	if(!tree.cloud.size()){
		return;
	}
	
	// Calculate height
	double max_height = tree.cloud.points[0].z;   
	double min_height = tree.cloud.points[0].z;
	int cloud_size = tree.cloud.height * tree.cloud.width;
	for(int i = 0; i< cloud_size; i++){	
		// find the min and max height
		if( tree.cloud.points[i].z > max_height) max_height = tree.cloud.points[i].z;
		else if( tree.cloud.points[i].z < min_height) min_height = tree.cloud.points[i].z;
	}
	tree.min_height = min_height;
	tree.max_height = max_height;
}

void TreeSegmentation::findMaximumTreesHeights(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  if(extracted_trunks_.size()==0)
    return;


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  int i=0;
  cloud_projected->resize(cloud.points.size());
  for(auto &point: cloud.points)
  {
    (*cloud_projected)[i].x = point.x;
    (*cloud_projected)[i].y = point.y;
    (*cloud_projected)[i].z = 0.0;
    i++;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  
  kdtree.setInputCloud (cloud_projected);

  int j = 0;
  int n_points = cloud.points.size();
  for( auto & trunk: extracted_trunks_)
  {
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      pcl::PointXYZ search_point;
      search_point.x = trunk.position[0];
      search_point.y = trunk.position[1];
      search_point.z = 0;
      
      float max_height = -1e10;
      float min_height = 1e10;
      float radius_search =  4.0;
      if ( kdtree.radiusSearch (search_point, radius_search, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
      {    
          for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
          {
            double dx = cloud[pointIdxRadiusSearch[i]].x - search_point.x;
            double dy = cloud[pointIdxRadiusSearch[i]].y - search_point.y;

            double dist_point = sqrt(dx*dx + dy+dy);            
            if(dist_point<=1.0)
              max_height = std::max(cloud[pointIdxRadiusSearch[i]].z, max_height);                 
            
            if(dist_point<=1.5)
              min_height = std::min(cloud[pointIdxRadiusSearch[i]].z, min_height);
          } 
            
      }
     
      trunk.max_height = max_height - min_height;
      trunk.position[2] = min_height;
      
      
      j=j+1; 

    }
  
}


void TreeSegmentation::generateUCM(std::vector<Tree> extracted_trunks_transformed, Eigen::Vector2d &local_frame, cv::Mat &ucm_image, double map_max_height)
{
  double grid_size = config_->grid_size_;
  double crown_radius = config_->crown_radius_;

  double x_min = 1000000;
  double y_min = 1000000;
  double x_max = -1000000;
  double y_max = -1000000;
  double space_offset = 5;

  for(auto & trunk: extracted_trunks_transformed)
  {
    x_min = (trunk.position[0] < x_min) ? trunk.position[0]: x_min;
    x_max = (trunk.position[0] > x_max) ? trunk.position[0]: x_max;
    y_min = (trunk.position[1] < y_min) ? trunk.position[1]: y_min;
    y_max = (trunk.position[1] > y_max) ? trunk.position[1]: y_max;
  }

  x_min-=space_offset;
  x_max+=space_offset;
  y_min-=space_offset;
  y_max+=space_offset;

  //Origin frame of images are on top left corner
  local_frame[0] = x_min;
  local_frame[1] = y_max;

  int image_width = (int)((x_max - x_min)/grid_size);
  int image_height = (int)((y_max - y_min)/grid_size);

  cv::Mat image(image_height,image_width, CV_8U, cv::Scalar(0));
  ucm_image = image.clone();


  for(auto &trunk: extracted_trunks_transformed)
  {
    
    double center_x = trunk.position[0];
    double center_y = trunk.position[1];

    int center_point_px_i = (int)((y_max - center_y)/grid_size);
    int center_point_px_j = (int)((center_x - x_min)/grid_size);

    cv::Point center(center_point_px_j,center_point_px_i);

    double point_x_min = center_x - crown_radius;
    double point_y_max = center_y + crown_radius;
    int point_px_i_min = (int)((y_max - point_y_max)/grid_size);
    int point_px_j_min = (int)((point_x_min - x_min)/grid_size);
    
    double point_x_max = center_x + crown_radius;
    double point_y_min = center_y - crown_radius;
    int point_px_i_max = (int)((y_max - point_y_min)/grid_size);
    int point_px_j_max = (int)((point_x_max - x_min)/grid_size);

    for(int i =  point_px_i_min-5; i<point_px_i_max+5; i++)
    {             
      for(int j =  point_px_j_min-5; j<point_px_j_max + 5; j++)
      {
        if(i>=0 && i<image_height && j>=0 && j<image_width)
        {
          double d_i = (i-center_point_px_i);
          double d_j = (j-center_point_px_j);
          double dist =  sqrt(d_i*d_i + d_j*d_j)*grid_size;
         
          int value = 0;

          if(map_max_height==0)
          {
            ROS_ERROR("Reference map maximum height no provided. UCM image cannot be computed.");
            return;
          }
          else
          {
            //Limit trees heights to map maximum height
            if(trunk.max_height>map_max_height)
              trunk.max_height = map_max_height;

            if(dist>=0 && dist<=crown_radius)
            {
              value = (int)(255*trunk.max_height/map_max_height);              
            }
              

          }
       
          ucm_image.at<uchar>(i,j) = (ucm_image.at<uchar>(i,j) >= value)? ucm_image.at<uchar>(i,j): value;
        }           
        
      }

    }   

  }

}