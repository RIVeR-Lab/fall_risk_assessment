#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

void activityCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv)
{

    typedef std::vector<std::string> frames;
    ros::init(argc, argv, "activity_tracker");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, activityCallback);

    ros::Publisher activity_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(30);

    tf::TransformListener listener;
    std::string openni_depth_frame;

    std::string dir = ros::package::getPath("activity_tracker");
    std::string file_path = dir+std::string("/conf/frames.cfg");
    std::ifstream fin(file_path.c_str());
    if (fin.fail()) {
        ROS_ERROR("Could not open %s.", file_path.c_str());
        exit(-1);
    }

    YAML::Node doc = YAML::Load(fin);
    frames target_frames;
    std::string reference_frame;
    try{
        target_frames = doc["target_frames"].as<frames>();
        reference_frame = doc["reference_frame"].as<std::string>();
    }
    catch(YAML::Exception& e){
        ROS_ERROR("Config file error!!! check conf/frames.cfg");
        exit(-1);
    }

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << count;

        for(unsigned i=0; i<target_frames.size(); i++){
            try{
                tf::StampedTransform transform;
                listener.lookupTransform(reference_frame, target_frames[i], ros::Time(0), transform);
                ss << " , "<<target_frames[i]<<" , "<<transform.getOrigin().getX()<<" , "
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

