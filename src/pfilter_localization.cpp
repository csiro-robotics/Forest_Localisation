////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "pfilter_localization.h"

PfilterLocalization::PfilterLocalization(ros::NodeHandle node, ros::NodeHandle private_node)
{

  node_ = node;
  private_node_ = private_node;

  config_.reset(new ConfigServer(node_,private_node_));
  visualisation_.reset(new Visualisation(node_,private_node_,config_));
  imu_interface_.reset(new ImuInterface(node_, private_node_,config_));
  tree_segmentation_.reset(new TreeSegmentation(node_, private_node_, config_));

  car_model = CarLikeModel(config_->wheels_axis_distance_);

  map_frame_ = config_->map_frame_;
  base_frame_ = config_->base_frame_;

  
  if(config_->num_threads_ > 0)
    pfilter_threads_.resize(config_->num_threads_);


  ucm_images_buffer_.resize(config_->numParticles_);
  local_frames_buffer_.resize(config_->numParticles_);

  last_odom_pose_.x = 0.0;
  last_odom_pose_.y = 0.0;
  last_odom_pose_.theta = 0.0;

  odom_pose_.x = 0.0;
  odom_pose_.y = 0.0;
  odom_pose_.theta = 0.0;

  robot_pose_.x = 0;
  robot_pose_.y = 0;
  robot_pose_.theta = 0;

  groundtruth_pose_.x = 0;
  groundtruth_pose_.y = 0;
  groundtruth_pose_.theta = 0;
  
  set_start_position_ = false;
  first_scan = true;

  random_gen.seed(random_dv());

  robot_moved_ = false;
 
  current_sweep_time_ = 0;
  
  first_gator_fb = true;


  string groundtruth_trajectory_file;
  groundtruth_trajectory_file = config_->root_folder_ +  config_->dataset_name_ + "/" + config_->groundtruth_trajectory_name_;

  if(groundtruth_trajectory_file.size()>0)
  {
    TrajectoryData::readTrajectoryFile(groundtruth_trajectory_file,config_->trajectory_rotation_,config_->trajectory_translation_, groundtruth_trajectory_);
  }
  else
    ROS_WARN("Groundtruth Trajectory not loaded: trajectory_file param not provided");

 
  laser_cloud_sub_ = node_.subscribe(config_->input_lidar_topic_, 100, &PfilterLocalization::laserCloudReceived, this);
  gator_info_sub_ = node_.subscribe(config_->gator_data_topic_,30, &PfilterLocalization::gatorInfoWheelOdom, this);
     
  publish_pose_ = node_.advertise<geometry_msgs::PoseStamped>("/pf/robot_pose",1);
  publish_gt_pose_ = node_.advertise<geometry_msgs::PoseStamped>("/pf/groundtruth_pose",1);
  publish_odom_pose_ = node_.advertise<geometry_msgs::PoseStamped>("/pf/odom",1);
  publish_max_weight_poses_ = node_.advertise<geometry_msgs::PoseArray>("/pf/particles/max_weight",1);

  publish_pose_path_ = node_.advertise<nav_msgs::Path>("/pf/robot/path",1);
  publish_groundtruth_path_ = node_.advertise<nav_msgs::Path>("/pf/groundtruth/path",1);
  publish_odom_pose_path_ = node_.advertise<nav_msgs::Path>("/pf/odom/path",1);
  
  publish_particles_update_ = node_.advertise<geometry_msgs::PoseArray>("/particles_update",1);
  publish_particles_resample_ = node_.advertise<geometry_msgs::PoseArray>("/particles_resample",1);

  if(config_->map_file_name_.size()>0)
  {
    std::string acm_file = config_->root_folder_ +  config_->dataset_name_ + "/" + config_->map_file_name_;
    image_map_.loadMapImage(acm_file, config_->map_image_frame_x_, config_->map_image_frame_y_, config_->grid_size_, config_->map_max_height_);  
  }
  else
    ROS_WARN("ACM Map image not provided");

  ROS_INFO("Pfilter node initialized");

}

PfilterLocalization::~PfilterLocalization()
{

}

void PfilterLocalization::setInitialPoses()
{
  if(set_start_position_)
  {         
    pose2D_t initial_pose(config_->initial_pose_[0], config_->initial_pose_[1], config_->initial_pose_[2]);
    
    odom_pose_.x = initial_pose.x;
    odom_pose_.y = initial_pose.y;
    odom_pose_.theta = initial_pose.theta;
    last_odom_pose_.x = initial_pose.x;
    last_odom_pose_.y = initial_pose.y;
    last_odom_pose_.theta = initial_pose.theta;

    printf("pose 0 = %.2f, %.2f \n", initial_pose.x, initial_pose.y);

    robot_pose_.x = initial_pose.x;
    robot_pose_.y = initial_pose.y;
    robot_pose_.theta = initial_pose.theta;
    
    InitParticles(initial_pose);
           
    set_start_position_ = false;

    ROS_INFO("Setting robot initial position");
            
  }
}

void PfilterLocalization::laserCloudReceived(const sensor_msgs::PointCloud2ConstPtr& laser_scan)
{
  bool generated_sweep = false;     
  generated_sweep = accumulateLidarScansWheel(laser_scan);
  
  if(generated_sweep) 
  {    
      extractTreeTrunks();   

      updateGroundTruthPose();
     
      //This is set only once
      setInitialPoses();

      //Check if robot has moved, if it has not, skip resampling step  
      pose2D_t pose_delta;
      pose_delta.x = odom_pose_.x - last_odom_pose_.x;
      pose_delta.y = odom_pose_.y - last_odom_pose_.y;
      pose_delta.theta = angle_diff(odom_pose_.theta,last_odom_pose_.theta);

      robot_moved_ = false;
      robot_moved_ = fabs(pose_delta.x) > config_->distance_thresh_ ||
      fabs(pose_delta.y) > config_->distance_thresh_ ||
      fabs(pose_delta.theta) > config_->angle_thresh_;

      PfilterAlgorithm();
      
      publishMsgs();
                 
      last_odom_pose_.x = odom_pose_.x;
      last_odom_pose_.y = odom_pose_.y;
      last_odom_pose_.theta = odom_pose_.theta;

    }


}

bool PfilterLocalization::accumulateLidarScansWheel(const sensor_msgs::PointCloud2ConstPtr& laser_scan)
{
    std::string laser_scan_frame_id = laser_scan->header.frame_id;
    current_laser_stamp_ = laser_scan->header.stamp;

    if(groundtruth_trajectory_.last_id>=groundtruth_trajectory_.transforms.size())
    {
      ROS_WARN("Trajectory vector reached end");
      return false;
    }
    
    tf2::Transform wheel_odom_transform;
    tf2::Transform final_transform;
    
    pcl::PointCloud<pcl::PointXYZ> cloud_out_pcl;
    pcl::PointCloud<pcl::PointXYZ> laser_scan_pcl;
    pcl::fromROSMsg(*laser_scan,laser_scan_pcl);
     
    tf2::Quaternion wheel_quat;
    wheel_quat.setRPY(0,0,odom_pose_.theta);

    wheel_odom_transform.setRotation(wheel_quat);
    wheel_odom_transform.setOrigin(tf2::Vector3(odom_pose_.x,odom_pose_.y,0));
    
    tf2::Transform gravity_transform;
    gravity_transform.setIdentity();

    if(config_->compensate_cloud_)
    {        
      sensor_msgs::Imu imu_data = imu_interface_->current_imu_;

      tf2::Quaternion imu_orient_q(imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w);
      tf2::Matrix3x3 imu_rot(imu_orient_q);
      tf2::Vector3 z_L(imu_rot[0].z(),imu_rot[1].z(),imu_rot[2].z());
      
      tf2Scalar roll, pitch, yaw;
      imu_rot.getRPY(roll, pitch, yaw);

      //printf("Roll, pitch, yaw: %0.2f, %0.2f, %0.2f \n", roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);

      tf2::Matrix3x3 rot_rp;
      rot_rp.setRPY(roll,pitch,0);
      
      tf2::Quaternion rot_quat;
      rot_rp.getRotation(rot_quat);

      gravity_transform.setOrigin(tf2::Vector3(0,0,0));      
      gravity_transform.setRotation(rot_quat);                
    
    }
    
    //Thansform input laser scans according to wheel odometry and gravity alignment (imu)
    final_transform = wheel_odom_transform*gravity_transform;
    geometry_msgs::Transform final_transform_msg;
    pcl_ros::transformPointCloud(laser_scan_pcl, cloud_out_pcl, tf2::toMsg(final_transform));

    current_sweep_time_ = laser_scan->header.stamp.toSec();
    if(first_scan)
    {            
      start_sweep_time_ = current_sweep_time_;
      first_scan = false;
      set_start_position_ = true;
    }

    //Accumulate point cloud data in a given window time = (laser_accum_time_)
    if(current_sweep_time_ - start_sweep_time_ <= config_->laser_accum_time_)
    {
      accumulated_cloud_pcl_+= cloud_out_pcl; 

      return false;                   
    }
    else{     
      
      pcl_ros::transformPointCloud(accumulated_cloud_pcl_,sweep_cloud_pcl_,tf2::toMsg(wheel_odom_transform.inverse()));
                    
      //reset sweep time counter
      start_sweep_time_ = laser_scan->header.stamp.toSec();
      current_sweep_time_ = start_sweep_time_;
      std::cout<<"Lidar accumulated points = "<<sweep_cloud_pcl_.points.size()<<std::endl;
      
      accumulated_cloud_pcl_.clear();
      
      return true;
    }
}

void PfilterLocalization::gatorInfoWheelOdom(const forest_localisation::gator_dataConstPtr &gator_msg)
{
  
  //avoid integrating small errors (close to zero) in instantaneous velocity
  if(gator_msg->inst_velocity >= config_->avg_velocity_thresh_)
  {
      if(first_gator_fb)
      {
        car_model.time = gator_msg->header.stamp.toSec();
        last_odom_pose_.x = 0.0;
        last_odom_pose_.y = 0.0;
        last_odom_pose_.theta = 0.0;
        odom_pose_.x = 0.0;
        odom_pose_.y = 0.0;
        odom_pose_.theta = 0.0;

        first_gator_fb = false;

      }
    
     
    car_model.v_linear = gator_msg->inst_velocity; //linear velocity in m/s
    car_model.steering_pos = -gator_msg->steering_angle;
    car_model.computeVelocities(odom_pose_.theta);

    car_model.updateOdometryPose(odom_pose_, gator_msg->header.stamp.toSec());    
  }

  car_model.time = gator_msg->header.stamp.toSec();

  geometry_msgs::PoseStamped odom_pose_msg;
  preparePoseMsg(odom_pose_,odom_pose_msg,map_frame_,gator_msg->header.stamp);
  publish_odom_pose_.publish(odom_pose_msg);

  robot_odom_path_.header.frame_id = map_frame_;
  
  robot_odom_path_.header.stamp = gator_msg->header.stamp;
  robot_odom_path_.poses.push_back(odom_pose_msg);
  publish_odom_pose_path_.publish(robot_odom_path_);
     
   
}

void PfilterLocalization::PfilterAlgorithm()
{    
    particle_state particle_state_temp;
    vector<particle_state> particles_temp;
    particles_temp.resize(config_->numParticles_);

    //Sampling from the robot motion model
  	for(int j = 0; j < config_->numParticles_; j++)
		{
		  particle_state_temp = sampleCarMotionModel(particles_[j]);	  
		
			particles_temp[j] = particle_state_temp;
		}
    particles_.clear();
    particles_= particles_temp;
  
    total_weight_ = 0;

    new_weights_.clear();
    new_weights_.resize(config_->numParticles_);

    std::vector<double> weights_copy;
    weights_copy.resize(config_->numParticles_);

    if(ucm_images_buffer_.size()!=0 && local_frames_buffer_.size()!=0)
    {
      ucm_images_buffer_.clear();
      local_frames_buffer_.clear();
      ucm_images_buffer_.resize(config_->numParticles_);
      local_frames_buffer_.resize(config_->numParticles_);
    }

   //If enough trees are detected construct UCM and perform Map Matching for each particle
   if(tree_segmentation_->extracted_trunks_.size()>=config_->min_landmarks_threshold_) 
   {  
      pfilter_threads_.clear();
      pfilter_threads_.resize(config_->num_threads_);
      int n_particles_thread = config_->numParticles_/config_->num_threads_;

      for (int id_thread = 0; id_thread<config_->num_threads_; id_thread++)
      {
        int init_id = n_particles_thread*id_thread;
        int end_id = std::min(n_particles_thread*id_thread + n_particles_thread-1, config_->numParticles_-1);
  
        pfilter_threads_[id_thread] = std::thread(&PfilterLocalization::mapMatchingThread,this, init_id, end_id);

      }

      for (auto it = pfilter_threads_.begin(); it != pfilter_threads_.end(); ++it) 
        it->join();
  
    }
    else
    {
      //In case few trees are detected (less than 2), weights from last measurement are kept, new weights will be multiplied by factor 1 
      for(int j = 0; j < config_->numParticles_; j++)
        new_weights_[j] = 1;
        
      total_weight_ = config_->numParticles_;

      ROS_INFO("Few trees detected - no resampling / keeping last particles weights");
    }
    

    for(int j = 0; j < config_->numParticles_; j++)
    {
      weights_copy[j] = new_weights_[j]/total_weight_;       
    }

    double has_nan = false;

    //Perform resampling of particles only if robot has moved and enough trees were detected
    if(robot_moved_ && tree_segmentation_->extracted_trunks_.size()>= config_->min_landmarks_threshold_)
    {
          
      for(int j = 0; j < config_->numParticles_; j++)
      {
        particles_[j].weight = new_weights_[j]/total_weight_; 
        if(std::isnan(particles_[j].weight))
          has_nan = true;        
      }
           
      LowVarianceSampler();
      total_weight_ = 1.0;
      
      ROS_INFO("Resampling occurred");
    }
    else
    {
      total_weight_ = 0;
      for(int j = 0; j < config_->numParticles_; j++)
      {
        float new_weight = particles_[j].weight*new_weights_[j];
        total_weight_+= new_weight;
        particles_[j].weight = new_weight; 
      }

      for(int j = 0; j < config_->numParticles_; j++)
      {
        particles_[j].weight /= total_weight_; 
        weights_copy[j] = particles_[j].weight; 
        if(std::isnan(particles_[j].weight))
          has_nan = true;
        
      }

    }

    //Get the 3 particles with highest weight (for visualisation purposes only) 
    particle_state highest_weight_particle;
    int high_particle_id;
    maximumParticleWeights(highest_weight_particle, high_particle_id, weights_copy);
    publishParticles(particles_temp, particles_);  

    double x,y,theta, c_theta, s_theta;
    x = 0;
    y = 0;
    c_theta = 0;
    s_theta = 0;
    total_weight_ = 0.0;

    //Compute estimated robot position using all particles (weighted average)
    for(int j = 0; j < config_->numParticles_; j++)
    {
        if(has_nan)
        {
          particles_[j].weight = 1.0/config_->numParticles_;          
        }   

        x = x + particles_[j].x * particles_[j].weight;
        y = y + particles_[j].y * particles_[j].weight;
        
        c_theta = c_theta + cos(particles_[j].theta)*particles_[j].weight;
        s_theta = s_theta + sin(particles_[j].theta)*particles_[j].weight;
        total_weight_+=particles_[j].weight;  
    }

    robot_pose_.x = x/(total_weight_);
    robot_pose_.y = y/(total_weight_);
    robot_pose_.theta = atan2(s_theta/total_weight_,c_theta/total_weight_);
   
    //Save robot position in the ACM map image frame (for visualisation purposes)
    Eigen::Vector2i robot_image_pos((robot_pose_.x - image_map_.image_frame_x_)/image_map_.grid_size_,(image_map_.image_frame_y_ - robot_pose_.y)/image_map_.grid_size_);
    robot_image_pos_.push_back(robot_image_pos);

     //Publish UCM image superimposed in ACM map image, but UCM image is translated and rotated according to particle position with highest weight
    if(tree_segmentation_->extracted_trunks_.size()>=config_->min_landmarks_threshold_)
    {
        visualisation_->publishMapAndBeliefImages(pose2D_t(highest_weight_particle.x,highest_weight_particle.y, highest_weight_particle.theta), 
          local_frames_buffer_[high_particle_id],
          ucm_images_buffer_[high_particle_id],
          image_map_,
          robot_image_pos_
        );
    }
 
}


void PfilterLocalization::extractTreeTrunks()
{
    if(sweep_cloud_pcl_.points.size() > 0)
    { 
      tree_segmentation_->extractTrunks(sweep_cloud_pcl_);    

      if(tree_segmentation_->extracted_trunks_.size()==0)
      {
        ROS_WARN("No trunks extracted");
        return;
      }

      ROS_INFO("Number of Extracted trunks: %d", int(tree_segmentation_->extracted_trunks_.size()));          

    }
    else{
      ROS_WARN("Trunk extraction callback didn't receive points");
    }
}


void PfilterLocalization::mapMatchingThread(int init_id, int end_id)
{ 
    for(int j = init_id; j < end_id+1; j++)
    {   
      double weight = 0;
    
      pose2D_t particle_pose;
      particle_pose.x = particles_[j].x;
      particle_pose.y = particles_[j].y;
      particle_pose.theta = particles_[j].theta;

      weight = mapMatchingParticle(particle_pose, j);
      
      data_assoc_mutex_.lock();
      new_weights_[j] = weight;
      total_weight_ += weight;
      data_assoc_mutex_.unlock();
    }
}


double PfilterLocalization::mapMatchingParticle(pose2D_t pose_particle, int particle_id)
{

  if(tree_segmentation_->extracted_trunks_.size()==0)
    return false;
 
  
  tf2::Quaternion temp_pose_q;
  temp_pose_q.setRPY(0, 0, pose_particle.theta);
  geometry_msgs::TransformStamped temp_pose_T;


  temp_pose_T.transform.translation.x = pose_particle.x;
  temp_pose_T.transform.translation.y  = pose_particle.y;
  temp_pose_T.transform.translation.z = 0.0;
  temp_pose_T.transform.rotation.x = temp_pose_q.getX();
  temp_pose_T.transform.rotation.y = temp_pose_q.getY();
  temp_pose_T.transform.rotation.z = temp_pose_q.getZ();
  temp_pose_T.transform.rotation.w = temp_pose_q.getW();
  temp_pose_T.header.frame_id = map_frame_;
  temp_pose_T.child_frame_id = "base_link";
  

  std::vector<Tree> extracted_trunks_transformed;

  extracted_trunks_transformed.resize(tree_segmentation_->extracted_trunks_.size());

  int i = 0;
  
  for(const auto& trunk: tree_segmentation_->extracted_trunks_) {

    geometry_msgs::PointStamped trunk_pos_in;
    trunk_pos_in.point.x = trunk.position.x();
    trunk_pos_in.point.y = trunk.position.y();
    trunk_pos_in.point.z = trunk.position.z();
    geometry_msgs::PointStamped trunk_pos_out;

    tf2::doTransform(trunk_pos_in,trunk_pos_out,temp_pose_T);

    Tree trunk_transformed;
    trunk_transformed.position[0] = trunk_pos_out.point.x;
    trunk_transformed.position[1] = trunk_pos_out.point.y;
    trunk_transformed.radius = trunk.radius;
    trunk_transformed.max_height = trunk.max_height;

    extracted_trunks_transformed[i] = trunk_transformed;
    i++;
  }

  cv::Mat ucm_image;

  Eigen::Vector2d local_frame;

  tree_segmentation_->generateUCM(extracted_trunks_transformed, local_frame, ucm_image, image_map_.max_height_);
     
  data_assoc_mutex_.lock();
  ucm_images_buffer_[particle_id] = ucm_image.clone();
  local_frames_buffer_[particle_id] = local_frame;
  data_assoc_mutex_.unlock();

  double grid_size = image_map_.grid_size_;
  double ssd_score = 0;
   
  cv::Point last_pos_px;
  last_pos_px.x = (pose_particle.x - image_map_.image_frame_x_)/grid_size;
  last_pos_px.y = (image_map_.image_frame_y_ - pose_particle.y)/grid_size;

  cv::Point last_pos_local_px;
  last_pos_local_px.x = (pose_particle.x - local_frame[0])/grid_size;
  last_pos_local_px.y = (local_frame[1] - pose_particle.y)/grid_size;

  if(last_pos_local_px.x < 0 || last_pos_local_px.y <0)
  {
    ROS_WARN("PARTICLE OUTSIDE IMAGE RANGE");
    return 0;
  }
  
  similarityMeasure(ucm_image, image_map_.image_, ssd_score, last_pos_px, last_pos_local_px, local_frame);
  
  Eigen::Vector2d map_image_frame(image_map_.image_frame_x_,image_map_.image_frame_y_);
  
  return ssd_score;  

}

void PfilterLocalization::similarityMeasure(cv::Mat &local_image, cv::Mat &map_image, double &score, cv::Point particle_pos_pxl, cv::Point particle_pos_local_pxl, Eigen::Vector2d local_frame)
{
  
  cv::Point template_corner;
  template_corner.x = std::max(particle_pos_pxl.x - particle_pos_local_pxl.x, 0);
  template_corner.y = std::max(particle_pos_pxl.y - particle_pos_local_pxl.y, 0);
  
  int diff_x = (particle_pos_pxl.x - particle_pos_local_pxl.x >= 0)? 0: (particle_pos_pxl.x - particle_pos_local_pxl.x);
  int diff_y = (particle_pos_pxl.y - particle_pos_local_pxl.y >= 0)? 0: (particle_pos_pxl.y - particle_pos_local_pxl.y);
  int dx = local_image.cols - diff_x;
  int dy = local_image.rows - diff_y;
  
  score = 0;
  double ssd_beta = config_->ssd_beta_;


  double cost = 0;
  
  int n_overlap = 0;
  float scale = image_map_.max_height_/255.0;
  for (int i = 0; i< dy ; i++)
  {
    for (int j=0; j< dx ; j++)      
    {      
      if((float)local_image.at<uchar>(i,j) > 0)
      {
        float diff = ((float)local_image.at<uchar>(i,j) - (float)map_image.at<uchar>(template_corner.y + i,template_corner.x + j))*scale;
        cost+= diff*diff;
        n_overlap++;
      }        
    }
  
  }
  
  cost = cost/(2*n_overlap);

  score = exp(-cost/ssd_beta);   
  
}

void PfilterLocalization::updateGroundTruthPose()
{
  int groundtruth_id_min =  TrajectoryData::findNearestTransformScan(groundtruth_trajectory_, config_->traj_search_window_, current_laser_stamp_, 0.01);
  tf2::fromMsg(groundtruth_trajectory_.transforms[groundtruth_id_min].transform, groundtruth_current_transform_);

  //Get real position (ground truth) from wildcat trajectory (.csv file)
  tf2::Vector3 gt_pos_vec = groundtruth_current_transform_.getOrigin();
  groundtruth_pose_.x = gt_pos_vec.x();
  groundtruth_pose_.y = gt_pos_vec.y();
  tf2::Matrix3x3 rot_groundtruth_pose(groundtruth_current_transform_.getRotation());
  tf2Scalar roll, pitch, yaw;
  rot_groundtruth_pose.getRPY(roll, pitch, yaw);
  groundtruth_pose_.theta = yaw;
  
}

void PfilterLocalization::InitParticles(pose2D_t initial_pose)
{
  printf("Initializing particles \n");
  for (int i=0; i<=config_->numParticles_; i++)
	{
		particle_state particle_temp;

		particle_temp.x =  rand() / (float)RAND_MAX * config_->particle_range_x_ + initial_pose.x - config_->particle_range_x_/2.0; 
		particle_temp.y = rand() / (float)RAND_MAX * config_->particle_range_y_ + initial_pose.y - config_->particle_range_y_/2.0;  



		particle_temp.theta = initial_pose.theta; 
		
		particle_temp.weight = 1.0/config_->numParticles_;   

    //printf("x: %4.2f y: %4.2f \n", particle_temp.x, particle_temp.y);
		particles_.push_back(particle_temp);  
	}
}

particle_state PfilterLocalization::sampleCarMotionModel(particle_state particle)
{
  
  float delta_trans1 = sqrt(pow((odom_pose_.x - last_odom_pose_.x),2) + pow((odom_pose_.y - last_odom_pose_.y),2));
  float delta_rot1 = 0.0; 
  
  //Compute rotation angle_1 only if the robot has moved a min distance (avoid inconsistence in atan2 calculation)
  if(delta_trans1>0.01)
    delta_rot1 = angle_diff(atan2(odom_pose_.y - last_odom_pose_.y,odom_pose_.x - last_odom_pose_.x), last_odom_pose_.theta);
  
  float delta_rot2 = angle_diff(angle_diff(odom_pose_.theta, last_odom_pose_.theta), delta_rot1);

	float delta_rot1_hat = angle_diff(delta_rot1, sampleNormalDistribution(config_->alpha1_*abs(delta_rot1) + config_->alpha2_*abs(delta_trans1)));
	float delta_trans1_hat  = delta_trans1 - sampleNormalDistribution(config_->alpha3_*delta_trans1 + config_->alpha4_*(abs(delta_rot1) + abs(delta_rot2)));
	float delta_rot2_hat  = angle_diff(delta_rot2, sampleNormalDistribution(config_->alpha1_*abs(delta_rot2) + config_->alpha2_*delta_trans1));

	particle_state particle_temp;

  particle_temp.x = particle.x + (delta_trans1_hat * cos(particle.theta + delta_rot1_hat));
  particle_temp.y = particle.y + (delta_trans1_hat * sin(particle.theta + delta_rot1_hat));
	particle_temp.theta = particle.theta + delta_rot1_hat + delta_rot2_hat;
	particle_temp.weight = particle.weight;

	return particle_temp;

}

double PfilterLocalization::sampleNormalDistribution(double std_dev)
{

  std::normal_distribution<double> distribution(0.0,std_dev);
  return distribution(random_gen);

}

void PfilterLocalization::LowVarianceSampler()
{
    vector<particle_state> particles_temp = particles_;
    float r = (rand() / (float)RAND_MAX) * (1.0 / (float)config_->numParticles_);
    float c = particles_[0].weight;
    int i = 0;

    for (int m = 0;m < config_->numParticles_; m++)
    {
        float u = r + (float) m/ config_->numParticles_;
      while (u > c && i < config_->numParticles_ - 1)
      { 
        i+=1;
        c += particles_temp[i].weight;	
      }
      particles_[m] = particles_temp[i];
      particles_[m].weight = 1.0 / config_->numParticles_;

    }
}
  
double PfilterLocalization::ProbNormal(double x, double std_dev)
{
  return (1/(std_dev*sqrt(2*M_PI)))*exp(-x*x/(2*std_dev*std_dev));
}
   
void PfilterLocalization::preparePoseMsg(pose2D_t pose, geometry_msgs::PoseStamped &msg, string frame, ros::Time stamp)   
{
      tf2::Quaternion q;
      q.setRPY(0,0,pose.theta);
      msg.pose.position.x = pose.x;
      msg.pose.position.y = pose.y;
      msg.pose.position.z = 0.0;
      msg.pose.orientation.x = q.getX();
      msg.pose.orientation.y = q.getY();
      msg.pose.orientation.z = q.getZ();
      msg.pose.orientation.w = q.getW();
      msg.header.frame_id = frame;
      msg.header.stamp = stamp;  
      
}

void PfilterLocalization::preparePoseMsg(pose2D_t pose, geometry_msgs::Pose &msg, string frame, ros::Time stamp)   
{
      tf2::Quaternion q;
      q.setRPY(0,0,pose.theta);
      msg.position.x = pose.x;
      msg.position.y = pose.y;
      msg.position.z = 0.0;
      msg.orientation.x = q.getX();
      msg.orientation.y = q.getY();
      msg.orientation.z = q.getZ();
      msg.orientation.w = q.getW();
      
}


void PfilterLocalization::publishMsgs()
{
      
      ros::Time time_now = ros::Time::now();

      visualisation_->publishDownsampledSweep(tree_segmentation_->filtered_cloud_,time_now);      
      visualisation_->publishClustersSegments(tree_segmentation_->extracted_trunks_,time_now);
      visualisation_->publishTrunksExtracted(tree_segmentation_->extracted_trunks_,time_now);
            
      ros::Time stamp = groundtruth_trajectory_.transforms[groundtruth_trajectory_.last_id].header.stamp;
      
      preparePoseMsg(robot_pose_,robot_pos_msg_,map_frame_,stamp);
      publish_pose_.publish(robot_pos_msg_);

      robot_pos_path_.header.frame_id = map_frame_;
      //robot_pos_path_.header.stamp = ros::Time::now();
      robot_pos_path_.header.stamp = stamp;
      robot_pos_path_.poses.push_back(robot_pos_msg_);
      publish_pose_path_.publish(robot_pos_path_);
    
      // Publish transform from robot frame to world frame
      static tf2_ros::TransformBroadcaster br;
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros::Time::now();
      //transformStamped.header.stamp = stamp ;
      transformStamped.header.frame_id = map_frame_;
      transformStamped.child_frame_id = config_->base_frame_;
      transformStamped.transform.translation.x = robot_pose_.x;
      transformStamped.transform.translation.y = robot_pose_.y;
      transformStamped.transform.translation.z = 0.0;
      tf2::Quaternion q_b;
      q_b.setRPY(0, 0, robot_pose_.theta);
      transformStamped.transform.rotation.x = q_b.x();
      transformStamped.transform.rotation.y = q_b.y();
      transformStamped.transform.rotation.z = q_b.z();
      transformStamped.transform.rotation.w = q_b.w();

      br.sendTransform(transformStamped);

      geometry_msgs::PoseStamped groundtruth_pose_msg;
      preparePoseMsg(groundtruth_pose_,groundtruth_pose_msg,map_frame_,stamp);
      publish_gt_pose_.publish(groundtruth_pose_msg);
    
      groundtruth_path_.header.frame_id = map_frame_;
      groundtruth_path_.header.stamp = stamp;
      groundtruth_path_.poses.push_back(groundtruth_pose_msg);
      publish_groundtruth_path_.publish(groundtruth_path_);
    
      geometry_msgs::PoseStamped odom_pose_msg;
      preparePoseMsg(odom_pose_,odom_pose_msg,map_frame_, stamp);
      publish_odom_pose_.publish(odom_pose_msg);

      robot_odom_path_.header.frame_id = map_frame_;
      robot_odom_path_.header.stamp = stamp;
      robot_odom_path_.poses.push_back(odom_pose_msg);
      publish_odom_pose_path_.publish(robot_odom_path_);
 
   
}

bool PfilterLocalization::saveData()
{ 
  string save_answer = "y"; 

  cout << "Would you like to save experiment data in files: yes (y), no (n)";
  
  cin >> save_answer;
  
  
  if(!save_answer.compare("y"))
  {
    

    cout<<"Starting to save data"<<endl;
    time_t now = time(0);
    tm *ltm = localtime(&now);
    stringstream  root_name;
    
    int year = ltm->tm_year + 1900;
    int month = ltm->tm_mon + 1;
    int day = ltm->tm_mday;
    int hour = ltm->tm_hour;
    int min = ltm->tm_min;
    int sec = ltm->tm_sec;
    
    root_name<< year;

    if(month<10)
      root_name<<"0";
    root_name<<month;

    if(day<10)
      root_name<<"0";
    root_name<<day;
    root_name<<"_";

    if(hour<10)
      root_name<<"0";
    root_name<<hour;

    if(min<10)
      root_name<<"0";
    root_name<<min;

    if(sec<10)
      root_name<<"0";
    root_name<<sec;

    string similarity_experiment_file = config_->save_folder_ + "similarity_experiment_data.txt";
    string time_files_names_file = config_->save_folder_ + "runtime_files.txt";

    config_->save_folder_ = config_->save_folder_ + "/" + root_name.str();
    if (mkdir(config_->save_folder_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
    {
        cerr << "Not possible to save files - Error when creating folder:  " << strerror(errno) << endl;
        return false;
    }
    config_->save_folder_= config_->save_folder_ + "/";
  

    string odom_file = config_->save_folder_ + root_name.str() + "_odom_pos.txt";
    string robot_file = config_->save_folder_ + root_name.str() + "_robot_pos.txt";
    string groundtruth_file = config_->save_folder_ + root_name.str() + "_groundtruth_pos.txt";
    
    string odom_py_file = config_->save_folder_ + root_name.str() + "_odom_pos_py.txt";
    
    string particles_file = config_->save_folder_ + root_name.str() + "_particles.txt";
    
    string info_file = config_->save_folder_ + root_name.str() + "_INFO.txt";
    
    string time_info_file = config_->save_folder_ + "runtime_info.txt";

    std::ofstream myFile(odom_file);

    if(!myFile.is_open())
    {
      cout<<"Error: Data not saved in files."<<endl;
      return false;
    }
    string header = "#timestamp x y z q_x q_y q_z q_w";
    myFile<<header<<"\n";
    int n_poses = robot_odom_path_.poses.size();
    int i = 0;

    for(const auto& odom_item: robot_odom_path_.poses) {
        myFile << odom_item.header.stamp << " ";
        myFile << odom_item.pose.position.x << " ";
        myFile << odom_item.pose.position.y << " ";
        myFile << odom_item.pose.position.z << " ";
        myFile << odom_item.pose.orientation.x << " ";
        myFile << odom_item.pose.orientation.y << " ";
        myFile << odom_item.pose.orientation.z << " ";
        myFile << odom_item.pose.orientation.w;
        if(i<n_poses-1)
          myFile<< "\n";
        i+=1;

    }
    myFile.close();


    myFile.open(odom_py_file);

    if(!myFile.is_open())
    {
      cout<<"Error: Data not saved in files."<<endl;
      return false;
    }
    
    string py_header = "#timestamp x y z yaw";
    myFile<<py_header<<"\n";
    n_poses = robot_odom_path_.poses.size();
    i = 0;
    float yaw = 0;
    for(const auto& odom_item: robot_odom_path_.poses) {
        myFile << odom_item.header.stamp << " ";
        myFile << odom_item.pose.position.x << " ";
        myFile << odom_item.pose.position.y << " ";
        myFile << odom_item.pose.position.z << " ";
        yaw = tf2::getYaw(odom_item.pose.orientation);
        myFile << yaw;
        if(i<n_poses-1)
          myFile<< "\n";
        i+=1;

    }
    myFile.close();


    myFile.open(robot_file); 

    if(!myFile.is_open())
    {
      cout<<"Error: Data not saved in files."<<endl;
      return false;
    }

    myFile<<header<<"\n";

    i = 0;
    n_poses = robot_pos_path_.poses.size();
    for(const auto& robot_item: robot_pos_path_.poses) {
        myFile << robot_item.header.stamp << " ";
        myFile << robot_item.pose.position.x << " ";
        myFile << robot_item.pose.position.y << " ";
        myFile << robot_item.pose.position.z << " ";
        myFile << robot_item.pose.orientation.x << " ";
        myFile << robot_item.pose.orientation.y << " ";
        myFile << robot_item.pose.orientation.z << " ";
        myFile << robot_item.pose.orientation.w;
        if(i<n_poses-1)
          myFile<< "\n";
        i+=1;
    }
    myFile.close();
    
    
    myFile.open(groundtruth_file);
    if(!myFile.is_open())
    {
      cout<<"Error: Data not saved in files."<<endl;
      return false;
    }

    myFile<<header<<"\n";
    
    i = 0;
    n_poses = groundtruth_path_.poses.size();
    for(const auto& robot_real_item: groundtruth_path_.poses) {
        myFile << robot_real_item.header.stamp << " ";
        myFile << robot_real_item.pose.position.x << " ";
        myFile << robot_real_item.pose.position.y << " ";
        myFile << robot_real_item.pose.position.z << " ";
        myFile << robot_real_item.pose.orientation.x << " ";
        myFile << robot_real_item.pose.orientation.y << " ";
        myFile << robot_real_item.pose.orientation.z << " ";
        myFile << robot_real_item.pose.orientation.w;
        if(i<n_poses-1)
          myFile<< "\n";
        i+=1;
    }

    myFile.close();
    
  
    myFile.open(info_file);
    if(!myFile.is_open())
    {
      cout<<"Error: Data not saved in files."<<endl;
      return false;
    }

    myFile<<"#Information file of experiment"<<"\n";

    myFile << "Oak Localization framework config data"<<"\n";
    myFile << config_->getAllParamsString()<<"\n";
      
    myFile.close();

    std::cout<<"Finished save files"<<std::endl;

    return true;
  }
  return false;

}


void PfilterLocalization::maximumParticleWeights(particle_state & highest_w_particle, int &particle_id, vector<double> particle_weights)
{
   vector<double> weights_copy = particle_weights;
    int max_weights_index [3];
    max_weights_index[0] = (int)(max_element(weights_copy.begin(), weights_copy.end()) - weights_copy.begin());
    weights_copy[ max_weights_index[0]] = 0.0;
    max_weights_index[1] = (int)(max_element(weights_copy.begin(), weights_copy.end()) - weights_copy.begin());
    weights_copy[ max_weights_index[1]] = 0.0;
    max_weights_index[2] = (int)(max_element(weights_copy.begin(), weights_copy.end()) - weights_copy.begin());
    
    geometry_msgs::PoseArray particles_max_weights_msg;

    particles_max_weights_msg.header.frame_id = map_frame_;
    particles_max_weights_msg.header.stamp = ros::Time::now(); 
    particles_max_weights_msg.poses.clear();
    geometry_msgs::Pose max_weight_pose;
    
    particle_state temp_particle = particles_[max_weights_index[0]];
    preparePoseMsg(pose2D_t(temp_particle.x,temp_particle.y, temp_particle.theta),max_weight_pose,map_frame_,ros::Time::now());
    particles_max_weights_msg.poses.push_back(max_weight_pose);
    
    highest_w_particle = temp_particle;
    particle_id = max_weights_index[0];
     
    temp_particle = particles_[max_weights_index[1]];
    preparePoseMsg(pose2D_t(temp_particle.x,temp_particle.y, temp_particle.theta),max_weight_pose,map_frame_,ros::Time::now());
    particles_max_weights_msg.poses.push_back(max_weight_pose);
    
    temp_particle = particles_[max_weights_index[2]];   
    preparePoseMsg(pose2D_t(temp_particle.x,temp_particle.y, temp_particle.theta),max_weight_pose,map_frame_,ros::Time::now());
    particles_max_weights_msg.poses.push_back(max_weight_pose);

    publish_max_weight_poses_.publish(particles_max_weights_msg);
      
    
}


void PfilterLocalization::publishParticles(vector<particle_state> particles, vector<particle_state> resampled_particles)
{ 

    particles_resample_msg_.poses.clear();
    particles_update_msg_.poses.clear();
    
    int divisor = 50;
  
    for(int j = 0; j < config_->numParticles_; j+=int(config_->numParticles_/divisor))
    {

        //prepare particle position messages
        geometry_msgs::Pose pose_temp;
        tf2::Quaternion q;  
        q.setRPY(0.0,0.0,particles[j].theta);
        pose_temp.position.x = particles[j].x;
        pose_temp.position.y = particles[j].y;
        pose_temp.position.z = 0.0;
        pose_temp.orientation.x = q.x();
        pose_temp.orientation.y = q.y();
        pose_temp.orientation.z = q.z();
        pose_temp.orientation.w = q.w();
        
        particles_update_msg_.header.frame_id = map_frame_;
        particles_update_msg_.header.stamp = ros::Time::now();
        particles_update_msg_.poses.push_back(pose_temp);

           //prepare particle position messages
        geometry_msgs::Pose pose_temp2;
        tf2::Quaternion q2;  
        q2.setRPY(0.0,0.0,resampled_particles[j].theta);
        pose_temp2.position.x = resampled_particles[j].x;
        pose_temp2.position.y = resampled_particles[j].y;
        pose_temp2.position.z = 0.0;
        pose_temp2.orientation.x = q2.x();
        pose_temp2.orientation.y = q2.y();
        pose_temp2.orientation.z = q2.z();
        pose_temp2.orientation.w = q2.w();
        
        particles_resample_msg_.header.frame_id = map_frame_;
        particles_resample_msg_.header.stamp = ros::Time::now();
        particles_resample_msg_.poses.push_back(pose_temp2);
         //particles_update_msg_.poses.push_back(pose_temp2);
    }

    publish_particles_resample_.publish(particles_resample_msg_);
    publish_particles_update_.publish(particles_update_msg_);



}

