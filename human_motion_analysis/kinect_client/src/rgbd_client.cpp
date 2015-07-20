// Ben Suay

// RGBD Client code that connects to the rgbd_server that runs
// on Windows 8.1 over TCP/IP. 

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

#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Socket Buffer Size
#define MAXDATASIZE 6553600

int main(int argc, char** argv)
{

  // Default port for rgbd service
  char port[] = "29015";

  // Default IP address for rgbd server
  char ip[] = "10.0.0.67";

  // number of bytes read from socket buffer (at a given recv() call)
  int numbytes;

  // number of bytes read from socket buffer (in total, to keep track of RGBD size)
  int totalNumBytesReceived;

  // we get this with the array
  unsigned long compressedSize;

  // If we can't read the RGBD at once we count remaining bytes to append
  // out readings. We calculate this by subtracting total bytes received
  // from compressed RGBD size.
  int remainingBytes;
 
  // This is fixed on the Windows 8.1 server side for now, but doesn't
  // have to be. 1920*1080*4 + 512*424*2 = 8728576
  unsigned long uncompressedSize = 8728576;

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
  ros::init(argc, argv, "kinect_rgbd_client");
  
  // This is our node handle
  ros::NodeHandle n;

  image_transport::ImageTransport it(n);
  
  // This publisher will publish RGB images
  image_transport::Publisher rgbPub = it.advertise("kinect_rgb", 1);
  
  // This publisher will publish Depth images
  image_transport::Publisher depthPub = it.advertise("kinect_depth", 1);

  cv_bridge::CvImage rgbMsg;
  cv_bridge::CvImage depthMsg;

  rgbMsg.encoding = sensor_msgs::image_encodings::BGRA8;
  depthMsg.encoding = sensor_msgs::image_encodings::MONO16;
  
  // How frequently do we want to publish our pointclouds? In Hz.
  ros::Rate loop_rate(100);
  
    
  // This flag defines the end of RGBD to be received from socket buffer. 
  // Otherwise we don't really know where the RGBD data would end in the buffer
  bool eorgbd = false;

  // Get RGBD from Kinect server
  if( myClient.Connect() == 0 )
    {
      while (ros::ok())
	{
	  
	  totalNumBytesReceived = 0; 

	  while( !eorgbd )
	    {
	      
	      // Receive data
	      numbytes = recv(myClient.s,buf,4,0);
	      
	      // debug
	      // std::cout << "Received " << numbytes << " this iteration" << std::endl;
	      // std::cout << "Received " << totalNumBytesReceived << " bytes in total" << std::endl;

	      memcpy(&compressedSize, buf, 4);	    

	      // debug
	      // std::cout << "I need to read " << compressedSize << " bytes." << std::endl;

	      char* myCompressedRGBD = new char[compressedSize];

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

		  // Get the rest of this block into compressed rgbd data
		  memcpy(myCompressedRGBD + totalNumBytesReceived, buf, numbytes);

		  // Increment the amount of bytes that we have read
		  totalNumBytesReceived += numbytes;  

		  // Update the amount of bytes remaining to read
		  remainingBytes = compressedSize - totalNumBytesReceived;

		  // debug
		  // std::cout << "Received " << totalNumBytesReceived << " bytes in total" << std::endl;
		  // std::cout << "Remaining bytes " << remainingBytes << " bytes in total" << std::endl;
		  
		}

	      // End of RGBD.
	      eorgbd = true;

	      // Allocate memory for decompressed data
	      char* pchDeCompressed = new char[uncompressedSize];
	      // unsigned char* img = new unsigned char[1920*1080*4];
	      cv::Mat myRGBImg(1080,1920,CV_8UC4);
	      cv::Mat myDepthImg(424,512,CV_16UC1);

	      // debug
	      // std::cout << "decompressing: " << compressedSize << " bytes." << std::endl;

	      // Decompress RGBD into allocated memory
	      LZ4_decompress_fast(myCompressedRGBD, pchDeCompressed, uncompressedSize);
	      
	      memcpy(myRGBImg.data, pchDeCompressed,1920*1080*4);
	      memcpy(myDepthImg.data, pchDeCompressed+1920*1080*4,512*424*2);

	      // msg.header.frame_id = i; --> IS THIS NECESSARY?
	      rgbMsg.header.stamp = ros::Time::now();
	      rgbMsg.image = myRGBImg;

	      depthMsg.header.stamp = ros::Time::now();
	      depthMsg.image = myDepthImg;

	      // debug
	      // cv::imwrite("myRGBImage.png",myRGBImg);
	      // cv::imwrite("myDepthImage.png",myDepthImg);

	      // Free allocated memory
	      delete[] pchDeCompressed;
	      delete[] myCompressedRGBD;

	      
	      // Just to make our intention clear:
	      // We won't be using these pointers until the next iteration
	      pchDeCompressed = NULL;
	      myCompressedRGBD = NULL;


	      // If the server closes connection exit the RGBD receiving loop
	      if( numbytes == 0 )
	       	{
	       	  eorgbd = true;
	       	  break;
	       	}
	    }	 
  
	  
	  // debug
	  // std::cout << "Processed data." << std::endl;
	  
	  // ready for the next iteration
	  eorgbd = false;

	  rgbPub.publish(rgbMsg.toImageMsg());
	  depthPub.publish(depthMsg.toImageMsg());
	  
		
	  // make it happen
	  ros::spinOnce ();

	  // If we wanted to throttle down the data rate
	  // This is the place to do so.
	  loop_rate.sleep ();
	}
    }
  
  
  return 0;
}
