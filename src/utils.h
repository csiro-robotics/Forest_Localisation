////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _PFILTER_UTILS_H
#define _PFILTER_UTILS_H

#include <math.h>
#include "ros/ros.h"
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include "geometry_msgs/TransformStamped.h"
#include "csv.h"
#include "tf2/utils.h"

#define SMALL_ANGLE 0.0002

struct pose2D_t
{
    pose2D_t(double x_in=0.0, double y_in=0.0, double theta_in=0.0){
        x = x_in;
        y = y_in;
        theta = theta_in;
    }
    double x;
    double y;
    double theta;
};

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

inline
std::string stripSlash(const std::string& in)
{
  std::string out = in;
  if ( ( !in.empty() ) && (in[0] == '/') )
    out.erase(0,1);
  return out;
}

static double positiveAngleDiff(double angle2, double angle1)
{
  if (angle1<0)
    angle1 = angle1 + 2*M_PI;

  if (angle2<0)
    angle2 = angle2 + 2*M_PI;
  
  double diff = angle2 - angle1;
  if(diff<0)
  {
      diff = diff + 2*M_PI;
  }
  return diff;
}

template <typename T> static void addParamToSstream(std::stringstream &ss, std::string name, T value)
{
  ss<<name << ":" << value <<"\n";
}

class TrajectoryData{
  
  public:
    
    double start_time;
    double end_time;
    std::vector<geometry_msgs::TransformStamped> transforms;
    int last_id;

    static bool readTrajectoryFile(const std::string traj_file, std::vector<float> init_rotation, std::vector<float> init_translation, TrajectoryData &trajectory)
    {
      io::CSVReader<8> in_gt(traj_file);
      //  int time_s, time_ns;
      double time;

      double x, y, z, qx, qy, qz, qw;


      in_gt.read_header(io::ignore_extra_column, "time","x","y","z","qw","qx","qy","qz");
      geometry_msgs::TransformStamped transform;

      transform.header.frame_id = "map";
      transform.child_frame_id = "base_link";

      ros::Rate r(10);
      bool init = true;
      while(in_gt.read_row(time, x, y, z, qw, qx, qy, qz))
      {
        if(init)
        {
          trajectory.start_time = time;
          init = false;
        }

        tf2::Transform trajectoryTransform;
        trajectoryTransform.setOrigin(tf2::Vector3(x,y,z));
        trajectoryTransform.setRotation(tf2::Quaternion(qx,qy,qz,qw));

        if(init_rotation.size() > 0 && init_translation.size() > 0)
        {
          tf2::Transform initTransform;
          initTransform.setOrigin(tf2::Vector3 (init_translation[0], init_translation[1], init_translation[2]));
          
          tf2::Matrix3x3 init_rot_matrix(init_rotation[0],init_rotation[1], init_rotation[2],
          init_rotation[3],init_rotation[4], init_rotation[5],
          init_rotation[6],init_rotation[7], init_rotation[8]);

          tf2::Quaternion init_quat;
          init_rot_matrix.getRotation(init_quat);

          initTransform.setRotation(init_quat);

          trajectoryTransform = initTransform*trajectoryTransform;
        }

        transform.transform.translation.x = trajectoryTransform.getOrigin()[0];
        transform.transform.translation.y = trajectoryTransform.getOrigin()[1];
        transform.transform.translation.z = trajectoryTransform.getOrigin()[2];

        transform.transform.rotation.w = trajectoryTransform.getRotation().getW();
        transform.transform.rotation.x = trajectoryTransform.getRotation().getX();
        transform.transform.rotation.y = trajectoryTransform.getRotation().getY();
        transform.transform.rotation.z = trajectoryTransform.getRotation().getZ(); 
        
        ros::Time pose_time;
        transform.header.stamp = pose_time.fromSec(time);

        trajectory.transforms.push_back(transform);

        trajectory.end_time = time;

      }

      trajectory.transforms.shrink_to_fit();
      trajectory.last_id = 0;
      return true;
    }

    static int findNearestTransformScan(TrajectoryData & trajectory, int search_window, ros::Time laser_stamp, double min_diff)
    {
      int start_indice = 0;

      for(int i=trajectory.last_id;i<trajectory.transforms.size(); i++)
      {
        ros::Duration diff=trajectory.transforms[i].header.stamp - laser_stamp;
        if(std::abs(diff.toSec())<=min_diff)
        {
          start_indice = i;
          break;
        }

      }

      int first = 0;
      if(start_indice - search_window > 0)
        first = start_indice - search_window;

      int id_min_diff = 0;
      double min_diff_time = 10.0;
      int end = first + 2*search_window;
      if(first + 2*search_window > trajectory.transforms.size())
        end = trajectory.transforms.size() -1;

      for(int i=first; i<=end; i++){
        auto traj_transform = trajectory.transforms[i];
        ros::Duration diff=traj_transform.header.stamp - laser_stamp;
          if(abs(diff.toSec())<=min_diff_time)
          {        
            min_diff_time = abs(diff.toSec());
            id_min_diff = i;
          }
      }
      if(id_min_diff==0)
        std::cout<<"ID before crash = "<<trajectory.last_id<<std::endl;
        
      trajectory.last_id = id_min_diff;

      return id_min_diff;
    
  }

    
};


#endif