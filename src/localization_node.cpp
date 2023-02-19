////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include "pfilter_localization.h"


int
main(int argc, char** argv)
{
  ros::init(argc, argv, "pfilter_localization_node");
  ros::NodeHandle n;

  ros::NodeHandle private_node = ros::NodeHandle ("~");

  PfilterLocalization pf_localization(n, private_node);

  while(ros::ok())
  {
      ros::Duration(0.1).sleep();
      ros::spinOnce();
          
  }
    
    pf_localization.saveData();

  

  return(0);
}
