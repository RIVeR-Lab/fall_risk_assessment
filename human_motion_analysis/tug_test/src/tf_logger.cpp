#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <ostream>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/stream.hpp>

namespace io = boost::iostreams;

void activityCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv)
{

    typedef std::vector<std::string> frames;

    ros::init(argc, argv, "tf_logger");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, activityCallback);

    ros::Publisher activity_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(30);

    tf::TransformListener listener;
    std::string openni_depth_frame;

    //Load config file
    std::string dir = ros::package::getPath("tug_test");
    std::string file_path = dir+std::string("/conf/frames.cfg");
    std::ifstream fin(file_path.c_str());
    if (fin.fail()) {
        ROS_ERROR("Could not open %s.", file_path.c_str());
        exit(-1);
    }

    //Read configuration
    YAML::Node doc = YAML::Load(fin);
    frames target_frames;
    frames reference_frame;
    try{
        target_frames = doc["target_frames"].as<frames>();
        reference_frame = doc["reference_frame"].as<frames>();
    }
    catch(YAML::Exception& e){
        ROS_ERROR("Config file error!!! check conf/frames.cfg, %s", e.what());
        exit(-1);
    }

    //using boost for writing tf data to csv file
    io::stream_buffer<io::file_sink> buf(dir+"/data/data.csv");
    std::ostream                     out(&buf);

    //create headers for csv file
    out<<"\"sequence_number\",";
    out<<"\"reference_frame\"";
    for(unsigned i=0; i<target_frames.size(); i++){
        out << ",\""<<target_frames[i]<<"_X\",";
        out << "\""<<target_frames[i]<<"_Y\",";
        out << "\""<<target_frames[i]<<"_Z\",";

        out << "\""<<target_frames[i]<<"_AngX\",";
        out << "\""<<target_frames[i]<<"_AngY\",";
        out << "\""<<target_frames[i]<<"_AngZ\",";
        out << "\""<<target_frames[i]<<"_AngW\"";
    }
    out<<std::endl;

    int count = 0;
    while (ros::ok())
    {
        //store requested transforms in csv file
        for(unsigned j=0; j<reference_frame.size(); j++){
            out << count;
            out<<",\""<<reference_frame[j]<<"\"";
            for(unsigned i=0; i<target_frames.size(); i++){
                try{
                    tf::StampedTransform transform;
                    listener.waitForTransform(reference_frame[j], target_frames[i], ros::Time(0), ros::Duration(3.0));
                    listener.lookupTransform(reference_frame[j], target_frames[i], ros::Time(0), transform);
                    out <<" , "<<transform.getOrigin().getX()<<" , "
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
            out<<std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
