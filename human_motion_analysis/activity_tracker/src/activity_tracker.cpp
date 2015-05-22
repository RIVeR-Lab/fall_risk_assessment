#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include <sstream>

void activityCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "activity_tracker");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, activityCallback);

    ros::Publisher activity_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    tf::TransformListener listener;
    std::string openni_depth_frame;
    std::string targetFrames[]={"/head_1","/right_hip_1","/torso_1","/left_foot_1" };
    std::string reference_frame="/right_foot_1";

    n.getParam("camera_frame_id", openni_depth_frame);
    ROS_INFO("%s", openni_depth_frame.c_str());
    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << count;

        for(int i=0; i<4; i++){
            try{
                tf::StampedTransform transform;
                listener.lookupTransform(reference_frame, targetFrames[i], ros::Time(0), transform);
                ss << " , "<<targetFrames[i]<<" , "<<transform.getOrigin().getX()<<" , "
                   <<transform.getOrigin().getY()<<" , "
                   <<transform.getOrigin().getZ()<<" , "
                   <<transform.getRotation().getX()<<" , "
                   <<transform.getRotation().getY()<<" , "
                   <<transform.getRotation().getZ()<<" , "
                   <<transform.getRotation().getW();
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

        }

        msg.data = ss.str();
        activity_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}

