// Ben Suay

// PCL Client code that connects to the pcl_server that runs
// on Windows 8.1 over TCP/IP.
// 


#include <string.h> 
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <vector>
#include <math.h>
#include <fstream> // save file

#include "TCP.hpp" // TCP/IP connection
#include "lz4.h" // lossless compression

#include "ros/ros.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Socket Buffer Size
#define MAXDATASIZE 6553600

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


int main(int argc, char** argv)
{

  // Default port for pcl service
  char port[] = "28015";

  // Default IP address for pcl server
  char ip[] = "10.0.0.67";

  // number of bytes read from socket buffer (at a given recv() call)
  int numbytes;

  // number of bytes read from socket buffer (in total, to keep track of PCL size)
  int totalNumBytesReceived;

  // we get this with the array
  unsigned long compressedSize;

  // If we can't read the PCL at once we count remaining bytes to append
  // out readings. We calculate this by subtracting total bytes received
  // from compressed PCL size.
  int remainingBytes;
 
  // This is fixed on the Windows 8.1 server side for now, but doesn't
  // have to be. This is the size of the PCL in number of bytes.
  // Remember, each point has X, Y and Z values (each of which is a float).
  // (Each float is 4 bytes long).
  unsigned long uncompressedSize = 2605056;

  // This is our buffer to do our readings.
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

  // Get an instance of the TCP/IP Client
  TCP myClient = TCP(port, ip);
 
  // Initialize the ROS node
  ros::init(argc, argv, "kinect_pcl_client");
  
  // This is our node handle
  ros::NodeHandle n;
    
  // This publisher will publish kinect PCL that we receive from Windows 8.1
  ros::Publisher kinect_cloud_pub = n.advertise<PointCloud> ("kinect_cloud", 1);
  
  // This is the payload for kinect PCL
  PointCloud::Ptr kinectMsg (new PointCloud);
    
  // How frequently do we want to publish our pointclouds? In Hz.
  ros::Rate loop_rate(100);
  
  // In what TF should the kinect PCL be shown?
  kinectMsg->header.frame_id = "kinect_sensor";
    
  // This flag defines the end of PCL to be received from socket buffer. 
  // Otherwise we don't really know where the PCL data would end in the buffer
  bool eopcl = false;

  // Get PCL from Kinect server
  if( myClient.Connect() == 0 )
    {
      while (ros::ok())
	{
	  
	  totalNumBytesReceived = 0;
	  
	  float* myXYZ = new float[uncompressedSize];

	  while( !eopcl )
	    {
	      
	      // Receive data
	      numbytes = recv(myClient.s,buf,4,0);
	      
	      // debug
	      // std::cout << "Received " << numbytes << " this iteration" << std::endl;
	      // std::cout << "Received " << totalNumBytesReceived << " bytes in total" << std::endl;

	      memcpy(&compressedSize, buf, 4);	    

	      // debug
	      // std::cout << "I need to read " << compressedSize << " bytes." << std::endl;

	      char* myCompressedPCL = new char[compressedSize];

	      // Initialize total number of bytes received
	      totalNumBytesReceived = 0;

	      // Calculate how many more bytes we need to receive
	      remainingBytes = compressedSize - totalNumBytesReceived;

	      // If we need to receive more than zero bytes, then go ahead and read
	      while(remainingBytes > 0)
		{
		  // At this reading, how many bytes do we want to read?
		  int readThisManyBytes;
		  
		  if(remainingBytes > MAXDATASIZE-1)
		    {
		      readThisManyBytes = MAXDATASIZE-1;
		    }
		  else
		    {
		      readThisManyBytes = remainingBytes;
		    }

		  // Receive bytes
		  numbytes = recv(myClient.s,buf,readThisManyBytes,0);

		  // Get the rest of this block into compressed pcl data
		  memcpy(myCompressedPCL + totalNumBytesReceived, buf, numbytes);

		  // Increment the amount of bytes that we have read
		  totalNumBytesReceived += numbytes;  

		  // Update the amount of bytes remaining to read
		  remainingBytes = compressedSize - totalNumBytesReceived;

		  // debug
		  // std::cout << "Received " << totalNumBytesReceived << " bytes in total" << std::endl;
		  // std::cout << "Remaining bytes " << remainingBytes << " bytes in total" << std::endl;
		  
		}

	      // End of PCL.
	      eopcl = true;

	      // Allocate memory for decompressed data
	      char* pchDeCompressed = new char[uncompressedSize];

	      // debug
	      // std::cout << "decompressing: " << compressedSize << " bytes." << std::endl;

	      // Decompress PCL into allocated memory
	      LZ4_decompress_fast(myCompressedPCL, pchDeCompressed, uncompressedSize);

	      // Keep a copy of decompressed PCL
	      memcpy(myXYZ, pchDeCompressed, uncompressedSize);

	      // Free allocated memory
	      delete[] pchDeCompressed;
	      delete[] myCompressedPCL;
	      
	      // Just to make our intention clear:
	      // We won't be using these pointers until the next iteration
	      pchDeCompressed = NULL;
	      myCompressedPCL = NULL;
	      

	      // If the server closes connection exit the PCL receiving loop
	      if( numbytes == 0 )
	       	{
	       	  eopcl = true;
	       	  break;
	       	}
	    }	 
  
	  // Our PCL data: 217088 points in space
	  // number of floats: 651264 = uncompressedSize / 4 (because our PCL is an array of floats and each float is 4 bytes long)
	  // number of points: 217088 = number of floats / 3 (because each point has 3 floats: X,Y,Z)
	  
	  std::vector<pcl::PointXYZ> p (217088, pcl::PointXYZ(0.0,0.0,0.0) ); 

	  // for each float in our PCL do
	  for(int i=0; i < 651264; i += 3)
	    {
	      // Assign each float where it belongs
	      // Note to self: Can we do a better assignment here without using a for loop?
	      // by typecasting maybe? 
	      // or would something like this work?
	      // p.assign( myXYZ, myXYZ+651264 )
	      p[i/3].x = myXYZ[i];
	      p[i/3].y = myXYZ[i+1];
	      p[i/3].z = myXYZ[i+2];
	    }

	  // debug
	  // std::cout << "Processed data." << std::endl;
	  

	  // ready for the next iteration
	  eopcl = false;

	  // Assign XYZ points to PCL2 message points
	  kinectMsg->points.assign(p.begin(), p.end() );

	  // Free up memory
	  delete[] myXYZ;
	  myXYZ = NULL;	  
	  
	  // for publishing kinect PCL2
	  kinectMsg->header.stamp = ros::Time::now ().toNSec()/1e3;

	  // Finally publish the PCL
	  kinect_cloud_pub.publish(kinectMsg);

	  // make it happen
	  ros::spinOnce ();

	  // If we wanted to throttle down the data rate
	  // This is the place to do so.
	  loop_rate.sleep ();
	}
    }
  
  
  return 0;
}
