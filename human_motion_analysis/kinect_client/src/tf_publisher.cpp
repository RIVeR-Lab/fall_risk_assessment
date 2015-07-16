#include "ros/ros.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_tf_publisher");
  ros::NodeHandle n;
  tf::TransformBroadcaster tf_broadcaster;
  tf::Transform T0_kinect;
  static tf::TransformBroadcaster br;

  ros::Duration sleeper(75.0/1000.0);

  while( ros::ok() )
    {
      T0_kinect.setRotation(tf::Quaternion(tf::Vector3(1.0, 0.0, 0.0), M_PI*0.5)*tf::Quaternion(tf::Vector3(0.0, 1.0, 0.0), M_PI*0.5));
      T0_kinect.setOrigin(tf::Vector3(-1.6764, -1.2446, 0.9017));
      br.sendTransform(tf::StampedTransform(T0_kinect, ros::Time::now(), "world", "kinect_sensor"));
      sleeper.sleep();
    }
  
  return 0;
}
