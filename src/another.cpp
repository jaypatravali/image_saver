#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>                                                 // Added this
#include "boost/filesystem.hpp"




static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/stereo/left/image_raw/", 1, &ImageConverter::imageCb, this);
    cv::namedWindow(OPENCV_WINDOW);
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

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    static int image_count = 0;                                // added this
    std::stringstream sstream;                               // added this
    sstream << "/home/patraval/catkin_ws2/src/image_saver/images/my_image" << image_count << ".png" ;                  // added this
    ROS_ASSERT( cv::imwrite( sstream.str(),  cv_ptr->image ) );      // added this
    image_count++;                                      // added this

// const char* path = _filePath.c_str();
// boost::filesystem::path dir(path);
// if(boost::filesystem::create_directory(dir))
// {
//     std::cerr<< "Directory Created: "<<_filePath<<std::endl;
// }
  }

// void create_directory()
// {

//   boost::filesystem::path dir("path");

//   if(!(boost::filesystem::exists(dir))){
//       std::cout<<"Doesn't Exists"<<std::endl;

//       if (boost::filesystem::create_directory(dir))
//           std::cout << "....Successfully Created !" << std::end;
//   }


// }

};




int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}