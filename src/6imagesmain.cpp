#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stereo_processor.h"

// to remove after debugging


static const std::string OPENCV_WINDOW1 = "Image window1";
static const std::string OPENCV_WINDOW2 = "Image window2";

static const std::string OPENCV_WINDOW3 = "Image window3";
static const std::string OPENCV_WINDOW4 = "Image window4";

static const std::string OPENCV_WINDOW5 = "Image window5";
static const std::string OPENCV_WINDOW6 = "Image window6";

static const std::string OPENCV_WINDOW7 = "Image window7";
static const std::string OPENCV_WINDOW8 = "Image window8";

static const std::string OPENCV_WINDOW9 = "Image window9";
static const std::string OPENCV_WINDOW10 = "Image window10";

static const std::string OPENCV_WINDOW11 = "Image window11";
static const std::string OPENCV_WINDOW12 = "Image window12";

static const std::string OPENCV_WINDOW13 = "Image window13";
static const std::string OPENCV_WINDOW14 = "Image window14";


class StereoOdometer : public StereoProcessor
{

public:
   StereoOdometer(const std::string& transport) :  StereoProcessor(transport)
  {
    cv::namedWindow(OPENCV_WINDOW1);
    cv::namedWindow(OPENCV_WINDOW2);
    cv::namedWindow(OPENCV_WINDOW3);
    cv::namedWindow(OPENCV_WINDOW4);
    cv::namedWindow(OPENCV_WINDOW5);
    cv::namedWindow(OPENCV_WINDOW6);
  }

~StereoOdometer()
  {
  cv::destroyWindow(OPENCV_WINDOW1);
  cv::destroyWindow(OPENCV_WINDOW2);
  cv::destroyWindow(OPENCV_WINDOW3);
  cv::destroyWindow(OPENCV_WINDOW4);
  cv::destroyWindow(OPENCV_WINDOW5);
  cv::destroyWindow(OPENCV_WINDOW6);
  }
protected:


  void imageCallback(
              const sensor_msgs::ImageConstPtr& l_image_msg,
              const sensor_msgs::ImageConstPtr& r_image_msg,
              const sensor_msgs::ImageConstPtr& msg_ZBL,
              const sensor_msgs::ImageConstPtr& msg_ZBR,
              const sensor_msgs::ImageConstPtr& msg_ZFL,
              const sensor_msgs::ImageConstPtr& msg_ZFR)
  {

// void imageCallback(
//               const sensor_msgs::ImageConstPtr& l_image_msg,
//               const sensor_msgs::ImageConstPtr& r_image_msg,
//               const sensor_msgs::ImageConstPtr& msg_ZBL,
//               const sensor_msgs::ImageConstPtr& msg_ZBR,
//               const sensor_msgs::ImageConstPtr& msg_ZFL,
//               const sensor_msgs::ImageConstPtr& msg_ZFR,
//               const sensor_msgs::NavSatFix::ConstPtr&msg_fix,
//               const nav_msgs::Odometry::ConstPtr& msg_odom)
//   {
    ros::WallTime start_time = ros::WallTime::now();

    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
    cv_bridge::CvImageConstPtr ptr_ZBL ;
    cv_bridge::CvImageConstPtr ptr_ZBR ;
    cv_bridge::CvImageConstPtr ptr_ZFL ;
    cv_bridge::CvImageConstPtr ptr_ZFR ;

    l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::BGR8);
    r_cv_ptr = cv_bridge::toCvShare(r_image_msg,sensor_msgs::image_encodings::BGR8);


    ptr_ZBL =  cv_bridge::toCvShare(msg_ZBL,sensor_msgs::image_encodings::BGR8);
    ptr_ZBR =  cv_bridge::toCvShare(msg_ZBR,sensor_msgs::image_encodings::BGR8);

    ptr_ZFL =  cv_bridge::toCvShare(msg_ZFL,sensor_msgs::image_encodings::BGR8);
    ptr_ZFR =  cv_bridge::toCvShare(msg_ZFR,sensor_msgs::image_encodings::BGR8);



    // Update GUI Window
    cv::imshow(OPENCV_WINDOW1, l_cv_ptr->image);
    cv::imshow(OPENCV_WINDOW2, r_cv_ptr->image);
    cv::imshow(OPENCV_WINDOW3, ptr_ZBL->image);
    cv::imshow(OPENCV_WINDOW4, ptr_ZBR->image);
    cv::imshow(OPENCV_WINDOW5, ptr_ZFL->image);
    cv::imshow(OPENCV_WINDOW6, ptr_ZFR->image);
    cv::waitKey(3);

  }



};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_odometer");

  std::string transport = argc > 1 ? argv[1] : "raw";
  StereoOdometer odometer(transport);

  ros::spin();
  return 0;
}
