#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "checklist_status/ChecklistStatusSrv.h"

float light;
// Add new variables for other sensors

bool getChecklistStatus(checklist_status::ChecklistStatusSrv::Request  &req,
         checklist_status::ChecklistStatusSrv::Response &res)
{
    res.luminosity=light;
    ROS_INFO("Checklist update responded!");
    return true;
}

void getLuminosity(const std_msgs::Float32& msg)
{
  light = msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "checklist_status_server");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/sensors/luminosity", 1000, getLuminosity);
// Add new subscribers and callback functions for other sensors topic
  ros::ServiceServer service = nh.advertiseService("checklist_status", getChecklistStatus);
  ROS_INFO("Ready to provide checklist information:");
  ros::spin();

  return 0;
}
