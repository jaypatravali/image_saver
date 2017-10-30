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
    cv::namedWindow(OPENCV_WINDOW7);
    cv::namedWindow(OPENCV_WINDOW8);
    cv::namedWindow(OPENCV_WINDOW9);
    cv::namedWindow(OPENCV_WINDOW10);
    cv::namedWindow(OPENCV_WINDOW11);
    cv::namedWindow(OPENCV_WINDOW12);
    cv::namedWindow(OPENCV_WINDOW13);
    cv::namedWindow(OPENCV_WINDOW14);

  }

~StereoOdometer()
  {
  cv::destroyWindow(OPENCV_WINDOW1);
  cv::destroyWindow(OPENCV_WINDOW2);
  cv::destroyWindow(OPENCV_WINDOW3);
  cv::destroyWindow(OPENCV_WINDOW4);
  cv::destroyWindow(OPENCV_WINDOW5);
  cv::destroyWindow(OPENCV_WINDOW6);
  cv::destroyWindow(OPENCV_WINDOW7);
  cv::destroyWindow(OPENCV_WINDOW8);
  cv::destroyWindow(OPENCV_WINDOW9);
  cv::destroyWindow(OPENCV_WINDOW10);
  cv::destroyWindow(OPENCV_WINDOW11);
  cv::destroyWindow(OPENCV_WINDOW12);
  cv::destroyWindow(OPENCV_WINDOW13);
  cv::destroyWindow(OPENCV_WINDOW14);

  }
protected:

  void imageCallback(
              const sensor_msgs::ImageConstPtr& l_image_msg,
              const sensor_msgs::ImageConstPtr& r_image_msg

              const sensor_msgs::ImageConstPtr& msg_ZBL,
              const sensor_msgs::ImageConstPtr& msg_ZBR,

              const sensor_msgs::ImageConstPtr& msg_ZFL,
              const sensor_msgs::ImageConstPtr& msg_ZFR,

              const sensor_msgs::ImageConstPtr& msg_ZFLL,
              const sensor_msgs::ImageConstPtr& msg_ZFLR,

              const sensor_msgs::ImageConstPtr& msg_ZFRL,
              const sensor_msgs::ImageConstPtr& msg_ZFRR,

              const sensor_msgs::ImageConstPtr& msg_ZBLL,
              const sensor_msgs::ImageConstPtr& msg_ZBLR,

              const sensor_msgs::ImageConstPtr& msg_ZBRL,
              const sensor_msgs::ImageConstPtr& msg_ZBRR)
  {
    ros::WallTime start_time = ros::WallTime::now();

    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
    cv_bridge::CvImageConstPtr ptr_ZBL ;
    cv_bridge::CvImageConstPtr ptr_ZBR ;
    cv_bridge::CvImageConstPtr ptr_ZFL ;
    cv_bridge::CvImageConstPtr ptr_ZFR ;
    cv_bridge::CvImageConstPtr ptr_ZFLL;
    cv_bridge::CvImageConstPtr ptr_ZFLR;
    cv_bridge::CvImageConstPtr ptr_ZFRL;
    cv_bridge::CvImageConstPtr ptr_ZFRR;
    cv_bridge::CvImageConstPtr ptr_ZBLL;
    cv_bridge::CvImageConstPtr ptr_ZBLR;
    cv_bridge::CvImageConstPtr ptr_ZBRL;
    cv_bridge::CvImageConstPtr ptr_ZBRR;


    l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::BGR8);
    r_cv_ptr = cv_bridge::toCvShare(r_image_msg,sensor_msgs::image_encodings::BGR8);


    ptr_ZBL =  cv_bridge::toCvShare(msg_ZBL,sensor_msgs::image_encodings::BGR8);
    ptr_ZBR =  cv_bridge::toCvShare(msg_ZBR,sensor_msgs::image_encodings::BGR8);

    ptr_ZFL =  cv_bridge::toCvShare(msg_ZFL,sensor_msgs::image_encodings::BGR8);
    ptr_ZFR =  cv_bridge::toCvShare(msg_ZFR,sensor_msgs::image_encodings::BGR8);

    ptr_ZFLL = cv_bridge::toCvShare(msg_ZFLL,sensor_msgs::image_encodings::BGR8);
    ptr_ZFLR = cv_bridge::toCvShare(msg_ZFLR,sensor_msgs::image_encodings::BGR8);

    ptr_ZFRL = cv_bridge::toCvShare(msg_ZFRL,sensor_msgs::image_encodings::BGR8);
    ptr_ZFRR = cv_bridge::toCvShare(msg_ZFRR,sensor_msgs::image_encodings::BGR8);

    ptr_ZBLL = cv_bridge::toCvShare(msg_ZBLL,sensor_msgs::image_encodings::BGR8);
    ptr_ZBLR = cv_bridge::toCvShare(msg_ZBLR,sensor_msgs::image_encodings::BGR8);

    ptr_ZBRL = cv_bridge::toCvShare(msg_ZBRL,sensor_msgs::image_encodings::BGR8);
    ptr_ZBRR = cv_bridge::toCvShare(msg_ZBRR,sensor_msgs::image_encodings::BGR8);



    // Update GUI Window
    cv::imshow(OPENCV_WINDOW1, l_cv_ptr->image);
    cv::imshow(OPENCV_WINDOW2, r_cv_ptr->image);
    cv::imshow(OPENCV_WINDOW3, ptr_ZBL->image);
    cv::imshow(OPENCV_WINDOW4, ptr_ZBR->image);
    cv::imshow(OPENCV_WINDOW5, ptr_ZFL->image);
    cv::imshow(OPENCV_WINDOW6, ptr_ZFR->image);
    cv::imshow(OPENCV_WINDOW7, ptr_ZFLL->image);
    cv::imshow(OPENCV_WINDOW8, ptr_ZFLR->image);
    cv::imshow(OPENCV_WINDOW9, ptr_ZFRL->image);
    cv::imshow(OPENCV_WINDOW10, ptr_ZFRR->image);
    cv::imshow(OPENCV_WINDOW11, ptr_ZBLL->image);
    cv::imshow(OPENCV_WINDOW12, ptr_ZBLR->image);
    cv::imshow(OPENCV_WINDOW13, ptr_ZBRL->image);
    cv::imshow(OPENCV_WINDOW14, ptr_ZBRR->image);

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
