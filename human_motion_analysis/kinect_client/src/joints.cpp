#include "TCP.hpp"
#include <string>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

#include <vector>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>
#include <boost/lexical_cast.hpp>

#define MAXDATASIZE 65536

int main(int argc, char** argv)
{

  char port[] = "27015";
  char ip[] = "10.0.0.67";

  int numbytes;
  char buf[MAXDATASIZE];

  // Get input parameters
  for(int a=0; a < argc; a++)
    {
      if( strcmp( argv[a], "--ip" ) == 0 )
	{
	  strcpy(ip, argv[a+1]);
	}

      if( strcmp( argv[a], "--port" ) == 0 )
	{
	  strcpy(port, argv[a+1]);
	}
    }

  TCP myClient = TCP(port, ip);

  ros::init(argc, argv, "joints");
  ros::NodeHandle n;

  std::string receivedString;
  std::string attribute;
  std::stringstream ss;
  std::istringstream iss;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate loop_rate(30);


  if( myClient.Connect() == 0 )
    {
      // Receive data
      numbytes = recv(myClient.s,buf,MAXDATASIZE-1,0);

      while (numbytes != 0 && ros::ok())
	{
	  numbytes = recv(myClient.s,buf,MAXDATASIZE-1,0);

	  buf[numbytes]='\0';

	  receivedString.assign(buf);

	  // debug
	  // std::cout << "Ahoy! Received " << numbytes << " bytes. " << receivedString << std::endl;

	  std::istringstream iss(receivedString);


	  std::vector<double> x (25, 0.0);
	  std::vector<double> y (25, 0.0);
	  std::vector<double> z (25, 0.0);

    std::vector<std::vector<double> > X (6, x);
    std::vector<std::vector<double> > Y (6, y);
    std::vector<std::vector<double> > Z (6, z);

    std::vector<double> qx (25, 0.0);
	  std::vector<double> qy (25, 0.0);
	  std::vector<double> qz (25, 0.0);
	  std::vector<double> qw (25, 1.0);

    std::vector<std::vector<double> > QX (6, qx);
    std::vector<std::vector<double> > QY (6, qy);
    std::vector<std::vector<double> > QZ (6, qz);
    std::vector<std::vector<double> > QW (6, qw);

	  int j;
	  int count = 6;
	  int id;

	  std::vector<int> foundIdx;
    std::string frame;
    std::string joint;

      /* code */

	  while(!iss.eof())
	    {
      // joint = "joint_";
      iss >> attribute;
      frame = "person_";

	      if( strcmp(attribute.c_str(), "count:") == 0 )
		{
		  iss >> count;
		  // ROS_INFO("count %d",count);
		}
	      else if( strcmp(attribute.c_str(), "id:") == 0 )
		{
		  iss >> id;
		  foundIdx.push_back(id);
      frame.append(boost::lexical_cast<std::string>(id));
		  // ROS_INFO("id %s",frame);
      // std::cout << id << " " << frame << " ";
		}
	      else if( strcmp(attribute.c_str(), "joint:") == 0 )
		{
		  iss >> j;
      //std::cout<<"found joint "<<j<<std::endl;
      // joint.append(boost::lexical_cast<std::string>(j));
      // joint.append(boost::lexical_cast<std::string>(j));
		  // ROS_INFO("joint %d",j);
		}
	      else if( strcmp(attribute.c_str(), "x:") == 0 )
		{
		  iss >> X[id][j];
		}
	      else if( strcmp(attribute.c_str(), "y:") == 0 )
		{
		  iss >> Y[id][j];
		}
	      else if( strcmp(attribute.c_str(), "z:") == 0 )
		{
		  iss >> Z[id][j];
		}


        else if( strcmp(attribute.c_str(), "qx:") == 0 )
        	{
        	  iss >> QX[id][j];
        	}
        else if( strcmp(attribute.c_str(), "qy:") == 0 )
        	{
        	  iss >> QY[id][j];
        	}
        else if( strcmp(attribute.c_str(), "qz:") == 0 )
        	{
        	  iss >> QZ[id][j];
        	}
        else if( strcmp(attribute.c_str(), "qw:") == 0 )
        	{
        	  iss >> QW[id][j];

        	}


      }




  for(int i = 0; i < j +1 ; i++){
      joint = "joint_";
      joint.append(boost::lexical_cast<std::string>(i));
      // std::cout<<"joint: "<<joint<<std::endl;

      // tf::Transform change_frame;
      // change_frame.setOrigin(tf::Vector3(0, 0, 0));
      // tf::Quaternion frame_rotation;
      // frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
      // change_frame.setRotation(frame_rotation);
      // transform = change_frame * transform;

      // kinect v2 doesn't give end point rotations.

      if (i == 3) {
        QX[id][i] = QX[id][2];
        QY[id][i] = QY[id][2];
        QZ[id][i] = QZ[id][2];
        QW[id][i] = QW[id][2];
      }

      if (i == 15) {
        QX[id][i] = QX[id][14];
        QY[id][i] = QY[id][14];
        QZ[id][i] = QZ[id][14];
        QW[id][i] = QW[id][14];
      }

      if (i == 19) {
        QX[id][i] = QX[id][18];
        QY[id][i] = QY[id][18];
        QZ[id][i] = QZ[id][18];
        QW[id][i] = QW[id][18];
      }

      if (i == 21 || i == 22) {
        QX[id][i] = QX[id][7];
        QY[id][i] = QY[id][7];
        QZ[id][i] = QZ[id][7];
        QW[id][i] = QW[id][7];
      }

      if (i == 23 || i == 24) {
        QX[id][i] = QX[id][11];
        QY[id][i] = QY[id][11];
        QZ[id][i] = QZ[id][11];
        QW[id][i] = QW[id][11];
      }



      transform.setOrigin( tf::Vector3(X[id][i], Y[id][i], Z[id][i]) );
      transform.setRotation( tf::Quaternion(QX[id][i], QY[id][i], QZ[id][i], QW[id][i]));
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "kinect_sensor", joint));

}

ros::spinOnce();
loop_rate.sleep();

	}

      // we're here if connection is closed
      std::cout << "----------------------CONNECTION CLOSED---------------------------"<< std::endl;
    }

  return 0;
}
