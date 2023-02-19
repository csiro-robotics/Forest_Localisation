#include "forest_localisation/gator_data.h"
#include "gator1hal/gator_feedback.h"

#include <ros/ros.h>

static ros::Publisher publish_gator_data_;

static void gatorReceivedCall(const gator1hal::gator_feedbackConstPtr &feedback_msg)
{
     forest_localisation::gator_data gator_msg;
    gator_msg.header.frame_id = "gator_base_link";
    gator_msg.header.seq = feedback_msg->header.seq;
    gator_msg.header.stamp = feedback_msg->header.stamp;
    gator_msg.inst_velocity = feedback_msg->inst_velocity;
    gator_msg.steering_angle = feedback_msg->steer_angle;
    publish_gator_data_.publish(gator_msg);
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "gator_handler_node");
    ros::NodeHandle n;

    ros::NodeHandle private_node = ros::NodeHandle ("~");

    publish_gator_data_ = n.advertise<forest_localisation::gator_data>("/gator_info",1);
    ros::Subscriber gator_feedback_sub_ = n.subscribe("gator1HAL/feedback",30, gatorReceivedCall);

    ros::spin();
    

  

    return(0);
}

