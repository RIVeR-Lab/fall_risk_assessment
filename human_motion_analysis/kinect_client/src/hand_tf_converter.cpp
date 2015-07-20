#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

ros::Publisher rightHandMarkerPub;
tf::TransformListener* listener=NULL;

void MarkerUpdate(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  int numMarkers = 24; // Number of links (i.e.t number of markers)
  size_t numSkeleton = msg->markers.size() / numMarkers; // Number of skeletons sent by kinect_client/skeleton_client node

  // For each skeleton, do
  for(size_t s=0; s < numSkeleton; s++)
    {
      // Check if this is the skeleton we're interested
      if( strcmp( msg->markers[s*numMarkers].ns.c_str(), "skeleton_id_1" ) == 0 )
	{
	  // Get the right hand marker location in kinect_sensor frame
	  tf::Vector3 handoverXYZ = tf::Vector3(msg->markers[s*numMarkers+10].points[1].x,
						msg->markers[s*numMarkers+10].points[1].y,
						msg->markers[s*numMarkers+10].points[1].z);
	  
	  tf::Transform TKinect_HandoverXYZ( tf::Quaternion(0,0,0,1),
					     handoverXYZ);

	  // Where in world is openrave transform and Kinect
	  tf::StampedTransform T0_Openrave;
	  tf::StampedTransform T0_Kinect;
	  tf::Transform TOpenrave_HandoverXYZ; // this is what we're trying to find out
	  visualization_msgs::Marker rightHandMarker; 

	  try
	    {
	      // Wait until openrave transform is available.
	      listener->waitForTransform("world", "openrave", ros::Time::now(), ros::Duration(0.1));
	      
	      // Look up the latest available transform in the buffer (which should theorethically be the same as now()
	      // but in practice we may have time extrapolation issues.
	      listener->lookupTransform("world", "openrave", ros::Time(0), T0_Openrave);

	      // Wait until kinect transform is available.
	      listener->waitForTransform("world", "kinect_sensor", ros::Time::now(), ros::Duration(0.1));
	      
	      // Look up the latest available transform in the buffer (which should theorethically be the same as now()
	      // but in practice we may have time extrapolation issues.
	      listener->lookupTransform("world", "kinect_sensor", ros::Time(0), T0_Kinect);

	      // Now find where handover point is wrt openrave transform
	      TOpenrave_HandoverXYZ = T0_Openrave.inverseTimes(T0_Kinect)*TKinect_HandoverXYZ;
	      
	      // Publish a marker for debugging
	      // debug
	      // Publish the handover pose with a marker (green sphere) for visualization in RViz
	      rightHandMarker.header.frame_id = "/openrave";
	      rightHandMarker.header.stamp = ros::Time::now();
	      rightHandMarker.ns = "rightHand";
	      rightHandMarker.id = 0;
	      rightHandMarker.type = visualization_msgs::Marker::SPHERE;
	      rightHandMarker.action = visualization_msgs::Marker::ADD;
	      rightHandMarker.scale.x = 0.05;
	      rightHandMarker.scale.y = 0.05;
	      rightHandMarker.scale.z = 0.05;
	      rightHandMarker.color.r = 1.0;
	      rightHandMarker.color.g = 0.0;
	      rightHandMarker.color.b = 0.0;
	      rightHandMarker.color.a = 1.0;
	      rightHandMarker.pose.position.x = TOpenrave_HandoverXYZ.getOrigin().x();
	      rightHandMarker.pose.position.y = TOpenrave_HandoverXYZ.getOrigin().y();
	      rightHandMarker.pose.position.z = TOpenrave_HandoverXYZ.getOrigin().z();
	      rightHandMarker.frame_locked = true;
	      
	      rightHandMarkerPub.publish(rightHandMarker);
	      
	    }
	  catch (tf::TransformException ex) 
	    {
	      ROS_ERROR("%s",ex.what());
	    }
	  
	  
	}
    }
}

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "hand_tf_converter");
  ros::NodeHandle n;
  listener = new tf::TransformListener(ros::Duration(180));
  // Subscribe to marker array
  ros::Subscriber sub = n.subscribe("joint_markers", 1, MarkerUpdate);
  rightHandMarkerPub = n.advertise<visualization_msgs::Marker> ("rightHand", 0);
  
  ros::spin();
  delete listener;
  return 0;
}
