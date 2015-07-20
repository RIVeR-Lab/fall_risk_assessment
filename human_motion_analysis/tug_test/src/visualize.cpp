#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/lexical_cast.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber activityState_sub_;
  std::string text_;
  ros::Time start_time;
  ros::Time stop_time;
  int elapsed_time;
  std::string time_;
  int currentAct;
  int previousAct;
  bool startCounter;

public:
  ImageConverter()
    : it_(nh_)
  {

    // Subscrive to input video feed and publish output video feed
   activityState_sub_ = nh_.subscribe("/activity_state", 1, &ImageConverter::getAct, this);
   image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this,image_transport::TransportHints("compressed"));

   //   image_sub_ = it_.subscribe("/camera/rgb/image",1,&ImageConverter::imageCb,this,image_transport::TransportHints("compressed"));
   image_pub_ = it_.advertise("/visualize/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
    startCounter = false;
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }





  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60){

      cv::putText(cv_ptr->image, text_ ,cv::Point(50,50), CV_FONT_HERSHEY_COMPLEX, 1, cvScalar(255, 0, 0));
      if (startCounter == true) {
        cv::putText(cv_ptr->image, time_ ,cv::Point(50,400), CV_FONT_HERSHEY_COMPLEX, 1, cvScalar(255, 0, 0));
      }

    }


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
  void getAct(const std_msgs::Int8::ConstPtr& msg){
    // text_ = msg->data.c_str();
    // ROS_INFO("%d", msg->data);

    if (msg->data == 1){
      text_ = "Standing";
      currentAct = msg->data;

    }
    else if (msg->data == 2){
      text_ = "Sitting";
      currentAct = msg->data;
    }

    if (previousAct == 2 && currentAct == 1 ) {
        start_time = ros::Time::now();
        startCounter = true;

    }
    else if (previousAct == 1 && currentAct == 2) {
      startCounter = false;
    }

    if (startCounter == true) {
        stop_time = ros::Time::now();
        elapsed_time  = stop_time.toSec() - start_time.toSec();
        time_ =  boost::lexical_cast<std::string>(elapsed_time);
        ROS_INFO("elapsed time: %s\n", time_.c_str());
    }


    previousAct = currentAct;

    }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize");
  ImageConverter ic;
  ros::spin();
  return 0;
}
