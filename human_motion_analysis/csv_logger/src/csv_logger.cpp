#include <iostream>
#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"


void logger_callback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    ROS_INFO("I heard :");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "csv_logger");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/tf", 50, logger_callback);
    ros::spin();


    return 0;
}
