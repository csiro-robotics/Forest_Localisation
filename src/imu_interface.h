////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Lucas C. Lima
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _IMU_INTERFACE_H
#define _IMU_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "config_server.h"

class ImuInterface
{
    public:

        ros::NodeHandle node_;
        ros::NodeHandle private_node_;
        std::shared_ptr<ConfigServer> config_;
        
        ros::Subscriber sub_imu;
    
        sensor_msgs::Imu last_imu_;
        sensor_msgs::Imu current_imu_;
        
        ros::Time current_time_;
        ros::Time last_time_;
       
        ImuInterface(ros::NodeHandle node, ros::NodeHandle private_node, std::shared_ptr<ConfigServer> &config): node_(node), private_node_(private_node)
        {
            config_ = config;
            sub_imu = node_.subscribe<sensor_msgs::Imu>  (config_->imu_topic_, 2000, &ImuInterface::ImuInterfaceSub,  this, ros::TransportHints().tcpNoDelay());              
        }


        void ImuInterfaceSub(const sensor_msgs::Imu::ConstPtr& imu_raw)
        {   
            last_imu_ = current_imu_;
            last_time_ = current_time_;
                    
            current_imu_ = *imu_raw;
            current_time_ = current_imu_.header.stamp;
        }


};


#endif