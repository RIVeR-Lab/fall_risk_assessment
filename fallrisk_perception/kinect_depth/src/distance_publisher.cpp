#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

namespace enc = sensor_msgs::image_encodings;
using namespace std;

//static const char WINDOW[] = "Image window";

class DistancePublisher
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher dist_pub_;

public:
  DistancePublisher()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/depth/image_rect", 1, &DistancePublisher::imageCb, this);
    dist_pub_ = nh_.advertise<std_msgs::Float32>("/distance/image_center_dist", 1);

    //cv::namedWindow(WINDOW);
  }

  ~DistancePublisher()
  {
    //cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //cout<<"rows="<<cv_ptr->image.rows<<endl;
    //cout<<"cols="<<cv_ptr->image.cols<<endl;
    /*
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    {
        //std::string txtLabel = str( boost::format("%d") % cv_ptr->image.at<float>(320,240));

        //cout<<txtLabel<<endl;

        cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2), 10, CV_RGB(255,0,0));
        cv::putText(cv_ptr->image, txtLabel, cv::Point(cv_ptr->image.cols/2+10, cv_ptr->image.rows/2-5), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 0, 0) );
    }
*/

    //cv::imshow(WINDOW, cv_ptr->image);
    //cv::waitKey(3);

    //cout<<"image type="<<cv_ptr->image.depth()<<endl;

    //cout<<"depth value="<<cv_ptr->image.at<float>(320,240)<<endl;

    std_msgs::Float32 msg_dist;

    msg_dist.data=cv_ptr->image.at<float>( cv_ptr->image.rows/2,cv_ptr->image.cols/2);
    //msg_dist.data=cv_ptr->image.at<float>( cv_ptr->image.cols/2,cv_ptr->image.rows/2);
    //msg_dist.data=0.0;

    cout<<"distance detected"<<endl;

    dist_pub_.publish(msg_dist);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dist_pub");
  DistancePublisher dp;
  ros::spin();
  return 0;
}
