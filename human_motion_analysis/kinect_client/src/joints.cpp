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
  int count1 = 0;
  int count2 = 0;

  if( myClient.Connect() == 0 )
    {
      // Receive data
      numbytes = recv(myClient.s,buf,MAXDATASIZE-1,0);

      while (numbytes != 0 && ros::ok())
	{
    count1++;
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

	  // No need anymore, but commented out just in case



	  // No need anymore, but commented out just in  case
	  // std::vector<std::vector<double> > QX (6, qx);
	  // std::vector<std::vector<double> > QY (6, qy);
	  // std::vector<std::vector<double> > QZ (6, qz);
	  // std::vector<std::vector<double> > QW (6, qw);

	  int j;
	  int count = 6;
	  int id;

	  // std::vector<double> orX (6, 0.0);
	  // std::vector<double> orY (6, 0.0);
	  // std::vector<double> orZ (6, 0.0);
	  // std::vector<double> orW (6, 0.0);

	  std::vector<int> foundIdx;
    std::string frame = "KinectV2_Person";
    std::string joint;

      /* code */

	  while(!iss.eof())
	    {
      // joint = "joint_";
      iss >> attribute;
      count2++;

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
// std::cout << "c1 : " << count1 << " c2 : " << count2 << std::endl;
  for(int i = 0; i < j; i++){
      joint = "joint_";
      joint.append(boost::lexical_cast<std::string>(i));
      // std::cout<<"joint: "<<joint<<std::endl;
      transform.setOrigin( tf::Vector3(X[id][i], Y[id][i], Z[id][i]) );
      transform.setRotation( tf::Quaternion(QX[id][i], QY[id][i], QZ[id][i], QW[id][i]));
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame, joint));

}

	}

      // we're here if connection is closed
      std::cout << "----------------------CONNECTION CLOSED---------------------------"<< std::endl;
    }

  return 0;
}
