#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "data_processor.h"
#include <math.h>

#include <iostream>
#include <fstream>

#include <chrono>
#include <thread>
#include <pthread.h>
#include <omp.h>

using namespace nav_msgs;
using namespace geometry_msgs;

double prev_x = 0; double prev_y = 0; double euc_distance= 0;

static const std::string OPENCV_WINDOW1 = "Image window1";
static const std::string OPENCV_WINDOW2 = "Image window2";
static const std::string OPENCV_WINDOW3 = "Image window3";
static const std::string OPENCV_WINDOW4 = "Image window4";
static const std::string OPENCV_WINDOW5 = "Image window5";
static const std::string OPENCV_WINDOW6 = "Image window6";

std::ofstream file("/export/patraval/robo_car_images/pg_cam/gps/fix.txt", std::ios::app);
//std::ofstream file2("/export/patraval/robo_car_images/zed_back/gps/fix.txt", std::ios::app);
//std::ofstream file3("/export/patraval/robo_car_images/zed_front/gps/fix.txt", std::ios::app);


std::ofstream file2("/export/patraval/robo_car_images/zed_rear_left/gps/fix.txt", std::ios::app);
std::ofstream file3("/export/patraval/robo_car_images/zed_rear_right/gps/fix.txt", std::ios::app);


static int image_count = 0;


std::vector<cv_bridge::CvImageConstPtr> image_left;
std::vector<cv_bridge::CvImageConstPtr> image_right;

cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
cv_bridge::CvImageConstPtr ptr_ZBL ;
cv_bridge::CvImageConstPtr ptr_ZBR ;
cv_bridge::CvImageConstPtr ptr_ZFL ;
cv_bridge::CvImageConstPtr ptr_ZFR ;

class DataWriter : public DataProcessor
{

public:


   DataWriter(const std::string& transport) :  DataProcessor(transport)
  {
    // cv::namedWindow(OPENCV_WINDOW1);
    // cv::namedWindow(OPENCV_WINDOW2);
    // cv::namedWindow(OPENCV_WINDOW3);
    // cv::namedWindow(OPENCV_WINDOW4);
    // cv::namedWindow(OPENCV_WINDOW5);
    // cv::namedWindow(OPENCV_WINDOW6);
  }

~DataWriter()
  {

  file.close(); 
  file2.close();
  file3.close();
  // cv::destroyWindow(OPENCV_WINDOW1);
  // cv::destroyWindow(OPENCV_WINDOW2);
  // cv::destroyWindow(OPENCV_WINDOW3);
  // cv::destroyWindow(OPENCV_WINDOW4);
  // cv::destroyWindow(OPENCV_WINDOW5);
  // cv::destroyWindow(OPENCV_WINDOW6);
  }
protected:


void imageCallback(
              const sensor_msgs::ImageConstPtr& l_image_msg,
              const sensor_msgs::ImageConstPtr& r_image_msg,
              const sensor_msgs::ImageConstPtr& msg_ZBL,
              const sensor_msgs::ImageConstPtr& msg_ZBR,
              const sensor_msgs::ImageConstPtr& msg_ZFL,
              const sensor_msgs::ImageConstPtr& msg_ZFR,
              const sensor_msgs::NavSatFix::ConstPtr&msg_fix,
              const nav_msgs::Odometry::ConstPtr& msg_odom)
  {
    const ros::WallTime& start_time = ros::WallTime::now();

    // l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::BGR8);
    // r_cv_ptr = cv_bridge::toCvShare(r_image_msg,sensor_msgs::image_encodings::BGR8);
    // ptr_ZBL =  cv_bridge::toCvShare(msg_ZBL,sensor_msgs::image_encodings::BGR8);
    // ptr_ZBR =  cv_bridge::toCvShare(msg_ZBR,sensor_msgs::image_encodings::BGR8);
    // ptr_ZFL =  cv_bridge::toCvShare(msg_ZFL,sensor_msgs::image_encodings::BGR8);
    // ptr_ZFR =  cv_bridge::toCvShare(msg_ZFR,sensor_msgs::image_encodings::BGR8);

    l_cv_ptr = cv_bridge::toCvCopy(l_image_msg, sensor_msgs::image_encodings::BGR8);
    r_cv_ptr = cv_bridge::toCvCopy(r_image_msg,sensor_msgs::image_encodings::BGR8);
    ptr_ZBL =  cv_bridge::toCvCopy(msg_ZBL,sensor_msgs::image_encodings::BGR8);
    ptr_ZBR =  cv_bridge::toCvCopy(msg_ZBR,sensor_msgs::image_encodings::BGR8);
    ptr_ZFL =  cv_bridge::toCvCopy(msg_ZFL,sensor_msgs::image_encodings::BGR8);
    ptr_ZFR =  cv_bridge::toCvCopy(msg_ZFR,sensor_msgs::image_encodings::BGR8);

    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW1, l_cv_ptr->image);
    // cv::imshow(OPENCV_WINDOW2, r_cv_ptr->image);
    // cv::imshow(OPENCV_WINDOW3, ptr_ZBL->image);
    // cv::imshow(OPENCV_WINDOW4, ptr_ZBR->image);
    // cv::imshow(OPENCV_WINDOW5, ptr_ZFL->image);
    // cv::imshow(OPENCV_WINDOW6, ptr_ZFR->image);
    // cv::waitKey(3);

    double x_current, y_current; 
    x_current = msg_odom->pose.pose.position.x;
    y_current = msg_odom->pose.pose.position.y;
    euc_distance = sqrt(pow((x_current- prev_x),2)+ pow((y_current- prev_y),2));

    image_left.push_back(l_cv_ptr); 
    image_right.push_back(r_cv_ptr); 
    image_left.push_back(ptr_ZBL );
    image_right.push_back(ptr_ZBR );
    image_left.push_back(ptr_ZFL );
    image_right.push_back(ptr_ZFR );


     // #pragma omp parallel for
     // for(int y=0; y <3; y++)
     //   {
     //     image_writer(image_left[y], image_right[y],msg_fix, msg_odom, y+1);
     //   }
     // const ros::WallTime& end_time = ros::WallTime::now();
     // ROS_INFO_STREAM("ELAPSED TIME: " << end_time.toSec() - start_time.toSec());
     // ROS_INFO_STREAM("Image Count " << image_count);
     // image_count++;        


    if ( euc_distance > 0.3 )
    {

     #pragma omp parallel for
     for(int y=0; y <3; y++)
       {
         image_writer(image_left[y], image_right[y],msg_fix, msg_odom, y+1);
       }
        // image_writer(l_cv_ptr, r_cv_ptr,msg_fix, msg_odom,1 );
        //image_writer(ptr_ZBL, ptr_ZBR,msg_fix, msg_odom, 2 );
       // image_writer(ptr_ZFL, ptr_ZFR,msg_fix, msg_odom, 3 );



     if( r_cv_ptr->image.empty() ) // Check for invalid input
     {
         std::cout <<  "Could not open or find the image" << std::endl ;
     }
    prev_x = x_current;
    prev_y = y_current;

     const ros::WallTime& end_time = ros::WallTime::now();
    
     ROS_INFO_STREAM("ELAPSED TIME: " << end_time.toSec() - start_time.toSec());
     ROS_INFO_STREAM("Image Count " << image_count);

    image_count++;        
    
    }

    image_left.clear();
    image_right.clear();
  }

    void image_writer(cv_bridge::CvImageConstPtr& l_cv_ptr, cv_bridge::CvImageConstPtr& r_cv_ptr, const sensor_msgs::NavSatFix::ConstPtr&msg_fix, const nav_msgs::Odometry::ConstPtr& msg_odom, int cam )
  {


    // static int image_count = 0;
    if (cam==1)                                
    {   std::stringstream sstream_left, sstream_right;
        // std::cout<<image_count<<"image_count"<<std::endl;
        sstream_left << "/export/patraval/robo_car_images/pg_cam/left/" << image_count << ".png" ;                  
        sstream_right << "/export/patraval/robo_car_images/pg_cam/right/" << image_count << ".png" ;   
        cv::imwrite( sstream_left.str(),  l_cv_ptr->image );    
        cv::imwrite( sstream_right.str(), r_cv_ptr->image ); 
        // file << image_count << std::endl;
        // file << *msg_odom << std::endl;
        // file << *msg_fix << std::endl;
        file << image_count <<" " << msg_fix-> latitude<< " " << msg_fix->longitude << " " << msg_odom-><< std::endl;
    }
    else if (cam==2)                                
    {   std::stringstream sstream_left, sstream_right;
        sstream_left << "/export/patraval/robo_car_images/zed_rear_left/left/" << image_count << ".png" ;                  
        sstream_right << "/export/patraval/robo_car_images/zed_rear_left/right/" << image_count << ".png" ;                  
        cv::imwrite( sstream_left.str(),  l_cv_ptr->image );    
        cv::imwrite( sstream_right.str(), r_cv_ptr->image );
        file2 << image_count << std::endl;
        file2 << *msg_odom << std::endl;
        file2 << *msg_fix << std::endl;

    }
    else if (cam==3)                                
    {   std::stringstream sstream_left, sstream_right;
        sstream_left << "/export/patraval/robo_car_images/zed_rear_right/left/" << image_count << ".png" ;                  
        sstream_right << "/export/patraval/robo_car_images/zed_rear_right/right/" << image_count << ".png" ; 
        cv::imwrite( sstream_left.str(),  l_cv_ptr->image );    
        cv::imwrite( sstream_right.str(), r_cv_ptr->image );      
        file3 << image_count << std::endl;
        file3 << *msg_odom << std::endl;
        file3 << *msg_fix << std::endl;

    }
                            
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Data_Writer");

  std::string transport = argc > 1 ? argv[1] : "raw";
  DataWriter writer(transport);

  ros::spin();
  return 0;
}



