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

    YAML::Emitter out;
    std::ofstream fout("file.yaml"); // take this file name as an argument

    typedef std::vector<std::string> frames;
    ros::init(argc, argv, "tf_logger");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, activityCallback);

    ros::Publisher activity_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(30);

    tf::TransformListener listener;
    std::string openni_depth_frame;

    std::string dir = ros::package::getPath("tug_test");
    std::string file_path = dir+std::string("/conf/frames.cfg");
    std::ifstream fin(file_path.c_str());
    if (fin.fail()) {
        ROS_ERROR("Could not open %s.", file_path.c_str());
        exit(-1);
    }

    YAML::Node doc = YAML::Load(fin);
    frames target_frames;
    frames reference_frame;
    try{
        target_frames = doc["target_frames"].as<frames>();
        ROS_INFO("Done 1");
        reference_frame = doc["reference_frame"].as<frames>();
        ROS_INFO("Done 2");
    }
    catch(YAML::Exception& e){
        ROS_ERROR("Config file error!!! check conf/frames.cfg, %s", e.what());
        exit(-1);
    }

    int count = 0;
    out << YAML::BeginMap;

    while (ros::ok())
    {

        std_msgs::String msg;
        for(unsigned j=0; j<reference_frame.size(); j++){
            // std::stringstream ss;
            // ss<<"," << count;
            // ss<<","<<reference_frame[j];
            out << YAML::Key << "reference_frame";
            out << YAML::Value << reference_frame[j];

            for(unsigned i=0; i<target_frames.size(); i++){
              out << YAML::BeginMap;
              out << YAML::Key << "target_frames";
              out << YAML::Value << target_frames[i];

                try{

                    tf::StampedTransform transform;
                    listener.lookupTransform(reference_frame[j], target_frames[i], ros::Time(0), transform);
                    // ss << " , "<<target_frames[i]<<" , "<<transform.getOrigin().getX()<<" , "
                    //    <<transform.getOrigin().getY()<<" , "
                    //   <<transform.getOrigin().getZ()<<" , "
                    //  <<transform.getRotation().getX()<<" , "
                    // <<transform.getRotation().getY()<<" , "
                    // <<transform.getRotation().getZ()<<" , "
                    // <<transform.getRotation().getW();

                      out << YAML::BeginMap;
                      out << YAML::Key << "positions";
                      out << YAML::Flow;
                      out << YAML::BeginSeq << transform.getOrigin().getX() << transform.getOrigin().getY() << transform.getOrigin().getZ() << YAML::EndSeq;
                      out << YAML::Key << "rotations";
                      out << YAML::Flow;
                      out << YAML::BeginSeq << transform.getRotation().getX() << transform.getRotation().getY() << transform.getRotation().getZ() << transform.getRotation().getW() << YAML::EndSeq;
                      out << YAML::EndMap;
                    


                    out << YAML::EndMap;

                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();

                  out << YAML::EndMap;
                }


            }



            // msg.data = ss.str();
            // activity_pub.publish(msg);

        }

        out << YAML::EndMap;
        fout << out.c_str();
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
