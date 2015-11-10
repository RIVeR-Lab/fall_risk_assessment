#include <iostream>
#include "ros/ros.h"
#include "ros/package.h"
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <tf/transform_listener.h>
#include <fstream>
#include <array>
#include <string>

using namespace std;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "csv_logger");
    ros::NodeHandle n;

    string target_frames[] = {"left_elbow_1", "right_elbow_1", "left_hand_1", "right_hand_1",
                              "left_knee_1", "right_knee_1", "left_foot_1", "right_foot_1"};
    string reference_frame = "torso_1";


    tf::TransformListener listener;

    int no_transforms = sizeof(target_frames)/sizeof(*target_frames);
    ros::Rate rate(10.0);
    double roll, pitch, yaw;
    tf::StampedTransform transform[no_transforms];

    //Initializing CSV file
    /*------------------------------------------------------------------------------------------------*/
    ofstream myfile;
    string current_directory = ros::package::getPath("csv_logger");

    myfile.open (current_directory +"/tf_logs/"+"user_transform_log.csv");
    myfile<<"REFERENCE FRAME," + reference_frame<<endl;
    myfile<<"TARGET FRAMES,";
    for(int i=0;i<no_transforms;i++)
    {
        myfile<<" , , ,"+ target_frames[i]+" , , , ,";
    }
    myfile<<"\n ,";
    for(int i=0;i<no_transforms;i++)
    {
        myfile<<"Quat_x, Quat_Y, Quat_Z, Quat_W, Dist_X, Dist_Y, Dist_Z,";
    }
    myfile<<endl;
    /*------------------------------------------------------------------------------------------------*/


    while (n.ok())
    {
        try
        {
          //listener.lookupTransform("left_elbow_1", "torso_1", ros::Time(0), transform);
          for(int i=0; i < no_transforms; i++)
          {
              listener.lookupTransform(target_frames[i], reference_frame, ros::Time(0), transform[i]);
              //ROS_INFO("size >>>>>>> =[%d]", sizeof(target_frames)/sizeof(*target_frames));
          }
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }

        ROS_INFO("X angle =[%f]", transform[0].getRotation().getX());
        ROS_INFO("Y angle =[%f]", transform[0].getRotation().getY());
        ROS_INFO("Z angle =[%f]", transform[0].getRotation().getZ());
        ROS_INFO("W       =[%f]", transform[0].getRotation().getW());
        ROS_INFO("X displacement =[%f]", transform[0].getOrigin().getX());
        ROS_INFO("Y displacement =[%f]", transform[0].getOrigin().getY());
        ROS_INFO("Z displacement =[%f]", transform[0].getOrigin().getZ());

        tf::Quaternion q(transform[0].getRotation().getX(), transform[0].getRotation().getY(),
                         transform[0].getRotation().getZ(), transform[0].getRotation().getW());
        tf::Matrix3x3 m(q);


        m.getRPY(roll, pitch, yaw);
        //std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
        ROS_INFO("roll =[%f]", roll);
        ROS_INFO("pitch =[%f]", pitch);
        ROS_INFO("yaw =[%f]", yaw);

        //Writing transforms to the file
        /*------------------------------------------------------------------------------------------------*/
        myfile<<" ,";
        for(int i=0; i < no_transforms; i++)
        {
            myfile<<transform[i].getRotation().getX()<<","<<transform[i].getRotation().getY()<<","<<
                    transform[i].getRotation().getZ()<<","<<transform[i].getRotation().getW()<<","<<
                    transform[i].getOrigin().getX()<<","<<transform[i].getOrigin().getY()<<","<<
                    transform[i].getOrigin().getZ()<<",";
        }

        myfile<<endl;
        /*------------------------------------------------------------------------------------------------*/

        rate.sleep();
      }

    ros::spin();


    return 0;
}


