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

  ros::init(argc, argv, "kinect_skeleton_client");
  ros::NodeHandle n;

  ros::Publisher g_marker_pub;
  ros::Publisher posePub;

  g_marker_pub = n.advertise<visualization_msgs::MarkerArray> ("joint_markers", 0);
  posePub = n.advertise<geometry_msgs::PoseArray> ("skeleton_joint_poses",0);

  visualization_msgs::MarkerArray markerArray;
  geometry_msgs::PoseArray poseArray;

  std::string receivedString;
  std::string attribute;
  std::stringstream ss;
  std::istringstream iss;

  static tf::TransformBroadcaster br;

  tf::StampedTransform T0_kinect;
  tf::Transform Tkinect_l5s1;

  std::map<int,int> skeletonLinks;
  std::map<int,std::string> jointType;
  std::map<int,int>::iterator it;

  ////////////////////////// Marker Index: marker.points[0] --> marker.points[1]
  //////////////////////////
  skeletonLinks[1] = 0;   // marker[0]:  spine base ------> spine mid
  skeletonLinks[20] = 1;  // marker[1]:  spine mid -------> shoulder mid
  skeletonLinks[2] = 20;  // marker[2]:  shoulder mid ----> neck
  skeletonLinks[3] = 2;   // marker[3]:  neck ------------> head
  skeletonLinks[4] = 20;  // marker[4]:  shoulder mid ----> shoulder left
  skeletonLinks[5] = 4;   // marker[5]:  shoulder left ---> elbow left
  skeletonLinks[6] = 5;   // marker[6]:  elbow left ------> wrist left
  skeletonLinks[22] = 6;  // marker[7]:  wrist left ------> thumb left
  skeletonLinks[7] = 6;   // marker[8]:  wrist left ------> hand left
  skeletonLinks[21] = 7;  // marker[9]:  hand left -------> hand tip left
  skeletonLinks[8] = 20;  // marker[10]: shoulder mid ----> shoulder right
  skeletonLinks[9] = 8;   // marker[11]: shoulder right --> elbow right
  skeletonLinks[10] = 9;  // marker[12]: elbow right -----> wrist right
  skeletonLinks[11] = 10; // marker[13]: wrist right -----> hand right
  skeletonLinks[23] = 11; // marker[14]: hand right ------> hand tip right
  skeletonLinks[24] = 10; // marker[15]: wrist right -----> thumb right
  skeletonLinks[12] = 0;  // marker[16]: spine base ------> hip left
  skeletonLinks[13] = 12; // marker[17]: hip left --------> knee left
  skeletonLinks[14] = 13; // marker[18]: knee left -------> ankle left
  skeletonLinks[15] = 14; // marker[19]: ankle left ------> foot left
  skeletonLinks[16] = 0;  // marker[20]: spine base ------> hip right
  skeletonLinks[17] = 16; // marker[21]: hip right -------> knee right
  skeletonLinks[18] = 17; // marker[22]: knee right ------> ankle right
  skeletonLinks[19] = 18; // marker[23]: ankle right -----> foot right
  //////////////////////////
  /////////////////////////////////////////////////////////////////////////////
  jointType[0] = "SpineBase";
  jointType[1] = "SpineMid";
  jointType[2] = "Neck";
  jointType[3] = "Head";
  jointType[4] = "ShoulderLeft";
  jointType[5] = "ElbowLeft";
  jointType[6] = "WristLeft";
  jointType[7] = "HandLeft";
  jointType[8] = "ShoulderRight";
  jointType[9] = "ElbowRight";
  jointType[10] = "WristRight";
  jointType[11] = "HandRight";
  jointType[12] = "HipLeft";
  jointType[13] = "KneeLeft";
  jointType[14] = "AnkleLeft";
  jointType[15] = "FootLeft";
  jointType[16] = "HipRight";
  jointType[17] = "KneeRight";
  jointType[18] = "AnkleRight";
  jointType[19] = "FootRight";
  jointType[20] = "SpineShoulder";
  jointType[21] = "HandTipLeft";
  jointType[22] = "ThumbLeft";
  jointType[23] = "HandTipRight";
  jointType[24] = "ThumbRight";

  tf::TransformListener T0_kinectListener;

  std::vector<float> R;
  std::vector<float> G;
  std::vector<float> B;

  R.push_back(1.0); G.push_back(0.0); B.push_back(0.0); // 0: red
  R.push_back(1.0); G.push_back(0.5); B.push_back(0.0); // 1: orange
  R.push_back(0.0); G.push_back(1.0); B.push_back(0.0); // 2: green
  R.push_back(0.0); G.push_back(1.0); B.push_back(1.0); // 3: cyan
  R.push_back(0.0); G.push_back(0.0); B.push_back(1.0); // 4: blue
  R.push_back(0.5); G.push_back(0.0); B.push_back(1.0); // 5: violet

  // static tf::TransformBroadcaster br;

  if( myClient.Connect() == 0 )
    {
      // Receive data
      numbytes = recv(myClient.s,buf,MAXDATASIZE-1,0);

      while (numbytes != 0)
	{

	  numbytes = recv(myClient.s,buf,MAXDATASIZE-1,0);

	  buf[numbytes]='\0';

	  receivedString.assign(buf);

	  // debug
	  // std::cout << "Ahoy! Received " << numbytes << " bytes. " << receivedString << std::endl;

	  std::istringstream iss(receivedString);

	  markerArray.markers.clear();
	  poseArray.poses.clear();

	  std::vector<double> x (25, 0.0);
	  std::vector<double> y (25, 0.0);
	  std::vector<double> z (25, 0.0);

	  // No need anymore, but commented out just in case
	  // std::vector<double> qx (25, 0.0);
	  // std::vector<double> qy (25, 0.0);
	  // std::vector<double> qz (25, 0.0);
	  // std::vector<double> qw (25, 1.0);

	  std::vector<std::vector<double> > X (6, x);
	  std::vector<std::vector<double> > Y (6, y);
	  std::vector<std::vector<double> > Z (6, z);

	  // No need anymore, but commented out just in  case
	  // std::vector<std::vector<double> > QX (6, qx);
	  // std::vector<std::vector<double> > QY (6, qy);
	  // std::vector<std::vector<double> > QZ (6, qz);
	  // std::vector<std::vector<double> > QW (6, qw);

	  int j;
	  int count = 6;
	  int id;

	  std::vector<double> orX (6, 0.0);
	  std::vector<double> orY (6, 0.0);
	  std::vector<double> orZ (6, 0.0);
	  std::vector<double> orW (6, 0.0);

	  std::vector<int> foundIdx;

	  while(!iss.eof())
	    {
	      iss >> attribute;

	      if( strcmp(attribute.c_str(), "count:") == 0 )
		{
		  iss >> count;
		  //ROS_INFO("count %d",count);
		}
	      else if( strcmp(attribute.c_str(), "id:") == 0 )
		{
		  iss >> id;
		  foundIdx.push_back(id);
		  //ROS_INFO("id %d",id);
		}
	      else if( strcmp(attribute.c_str(), "joint:") == 0 )
		{
		  iss >> j;
		  //ROS_INFO("joint %d",j);
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
	      else if( strcmp(attribute.c_str(), "orientation:") == 0 )
		{
		  // This is mid-spine joint oritentation in quaternion
		  iss >> orX[id];
		  iss >> orY[id];
		  iss >> orZ[id];
		  iss >> orW[id];
		}
	      // I had added the following block to use the
	      // orientation data that used to come from the
	      // windows server but now I chopped that part
	      // off since I don't need the data.
	      //
	      // else if( strcmp(attribute.c_str(), "qx:") == 0 )
	      // 	{
	      // 	  iss >> QX[id][j];
	      // 	}
	      // else if( strcmp(attribute.c_str(), "qy:") == 0 )
	      // 	{
	      // 	  iss >> QY[id][j];
	      // 	}
	      // else if( strcmp(attribute.c_str(), "qz:") == 0 )
	      // 	{
	      // 	  iss >> QZ[id][j];
	      // 	}
	      // else if( strcmp(attribute.c_str(), "qw:") == 0 )
	      // 	{
	      // 	  iss >> QZ[id][j];
	      // 	}

	    }

	  try
	    {
	      T0_kinectListener.lookupTransform("world", "kinect_sensor", ros::Time(0), T0_kinect);
	    }
	  catch (tf::TransformException ex)
	    {
	      ROS_ERROR("%s",ex.what());
	      ros::Duration(1.0).sleep();
	    }

	  // Publish only transforms that belong to bodies tracked by Kinect
	  for( size_t idx=0; idx < foundIdx.size(); idx++)
	    {
	      int c = foundIdx[idx];

	      //ROS_INFO("Processing %d",c);

	      Tkinect_l5s1.setRotation( tf::Quaternion(orX[c],orY[c],orZ[c],orW[c]) );
	      Tkinect_l5s1.setOrigin(tf::Vector3(X[c][20] - ( (X[c][20]-X[c][0])*0.8065 ), Y[c][20] - ( (Y[c][20]-Y[c][0])*0.8065 ), Z[c][20] - ( (Z[c][20]-Z[c][0])*0.8065 )));
	      std::stringstream indx;
	      indx << c;
	      br.sendTransform(tf::StampedTransform(Tkinect_l5s1, ros::Time::now(), "kinect_sensor", "l5s1_"+indx.str()));

	      // SHOW CUBES
	      //
	      // for(int i=0; i < 25; i++)
	      //   {
	      //     visualization_msgs::Marker marker;
	      //     marker.header.frame_id = "/kinect_sensor";
	      //     marker.header.stamp = ros::Time::now();
	      //     ss.str("");
	      //     ss << i;
	      //     marker.ns = "joint_marker_" + ss.str();
	      //     marker.id = i;
	      //     marker.type = visualization_msgs::Marker::CUBE;
	      //     marker.action = visualization_msgs::Marker::ADD;

	      //     marker.pose.position.x = x[i];
	      //     marker.pose.position.y = y[i];
	      //     marker.pose.position.z = z[i];

	      //     marker.pose.orientation.x = 0.0;
	      //     marker.pose.orientation.y = 0.0;
	      //     marker.pose.orientation.z = 0.0;
	      //     marker.pose.orientation.w = 1.0;
	      //     marker.scale.x = 0.03;
	      //     marker.scale.y = 0.03;
	      //     marker.scale.z = 0.03;
	      //     marker.color.r = 1;
	      //     marker.color.g = 0.5;
	      //     marker.color.b = 0.0;
	      //     marker.color.a = 1.0;
	      //     // marker.lifetime = lifetime;
	      //     marker.frame_locked = true;

	      //     markerArray.markers.push_back(marker);
	      //   }

	      // Publish joint normal transforms
	      // No need anymore but left it here just in case.
	      // for (int j=0; j < 25; j++)
	      // 	{
	      // 	  tf::Transform Tkinect_jointNormal;

	      // 	  Tkinect_jointNormal.setRotation(tf::Quaternion(QX[c][j], // set rotation
	      // 							 QY[c][j],
	      // 							 QZ[c][j],
	      // 							 QW[c][j])
	      // 					  );


	      // 	  Tkinect_jointNormal.setOrigin(tf::Vector3(X[c][j],
	      // 						    Y[c][j],
	      // 						    Z[c][j]));   // set origin

	      // 	  // Tkinect_jointNormal.setRotation(Tkinect_jointNormal.getRotation()*tf::Quaternion(tf::Vector3(0.0,0.0,-1.0),M_PI*0.5)*tf::Quaternion( tf::Vector3(0.0,-1.0,0.0), M_PI*0.5));

	      // 	  br.sendTransform( tf::StampedTransform( Tkinect_jointNormal, ros::Time::now(), "kinect_sensor", jointType[j]+"_"+idx.str() ) );
	      // 	}


	      // SHOW LINE LIST
	      int k=0;
	      for (it=skeletonLinks.begin(); it!=skeletonLinks.end(); ++it)
		{

		  visualization_msgs::Marker marker;
		  marker.header.frame_id = "/kinect_sensor";
		  marker.header.stamp = ros::Time::now();
		  ss.str("");
		  ss << c;
		  marker.ns = "skeleton_id_"+ ss.str();
		  marker.id = it->first;
		  marker.type = visualization_msgs::Marker::LINE_LIST;
		  marker.action = visualization_msgs::Marker::ADD;

		  marker.scale.x = 0.03;
		  marker.color.r = R[c];
		  marker.color.g = G[c];
		  marker.color.b = B[c];
		  marker.color.a = 1.0;

		  marker.frame_locked = true;

		  geometry_msgs::Point p0;
		  geometry_msgs::Point p1;

		  p0.x = X[c][it->second];
		  p0.y = Y[c][it->second];
		  p0.z = Z[c][it->second];

		  p1.x = X[c][it->first];
		  p1.y = Y[c][it->first];
		  p1.z = Z[c][it->first];

		  marker.points.push_back(p0);
		  marker.points.push_back(p1);

		  markerArray.markers.push_back(marker);

		  // Fill in the
		  // geometry_msgs::Pose pose;
		  // pose.position = p0;
		  // pose.orientation.x = QX[c][it->second];
		  // pose.orientation.y = QY[c][it->second];
		  // pose.orientation.z = QZ[c][it->second];
		  // pose.orientation.w = QW[c][it->second];
		  // poseArray.poses.push_back(pose);


		}

	      g_marker_pub.publish(markerArray);
	      //poseArray.header.frame_id = "kinect_sensor";
	      //poseArray.header.stamp = ros::Time::now();
	      //posePub.publish(poseArray);

	      //What do I do with the recived data
	      // Receive(numbytes,buf);
	    }
	}

      // we're here if connection is closed
      std::cout << "----------------------CONNECTION CLOSED---------------------------"<< std::endl;
    }

  return 0;
}
