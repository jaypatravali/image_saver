#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>



#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>


 int image_count = 0;
cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;

  // subscriber
image_transport::SubscriberFilter left_sub_, right_sub_;


typedef message_filters::sync_policies::ExactTime<  sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;

typedef message_filters::sync_policies::ApproximateTime<  sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;

typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
boost::shared_ptr<ExactSync> exact_sync_;
boost::shared_ptr<ApproximateSync> approximate_sync_;
int queue_size_;


void sync_callback(const sensor_msgs::ImageConstPtr& l_image_msg, const sensor_msgs::ImageConstPtr& r_image_msg){


    l_cv_ptr = cv_bridge::toCvCopy(l_image_msg, sensor_msgs::image_encodings::BGR8);
    r_cv_ptr = cv_bridge::toCvCopy(r_image_msg,sensor_msgs::image_encodings::BGR8);

     ROS_INFO_STREAM(" Processed: " << image_count);

    // cv::imshow("left", l_cv_ptr->image);
    // cv::imshow("right", r_cv_ptr->image);

    // cv::waitKey(3);
    std::stringstream sstream_left, sstream_right;
    sstream_left << "/export/patraval/robo_car_images/pg_cam/rect/left/" << image_count << ".png" ;                  
    sstream_right << "/export/patraval/robo_car_images/pg_cam/rect/right/" << image_count << ".png" ;   
    std::cout<<sstream_left.str()<<std::endl;
    cv::imwrite( sstream_left.str(),  l_cv_ptr->image );    
    cv::imwrite( sstream_right.str(), r_cv_ptr->image ); 
    image_count++;


}


int main(int argc, char **argv){

    ros::init(argc, argv, "Stereo_Saver");
    ros::NodeHandle nh;


    // Read local parameters
    ros::NodeHandle local_nh("~");


    std::string left_topic = ros::names::clean("/stereo/rect/first/image");
    std::string right_topic = ros::names::clean("/stereo/rect/second/image");

    // Subscribe to four input topics.
    ROS_INFO("Subscribing to:\n\t* %s\n\t*  %s\n\t*  ",
        left_topic.c_str(), right_topic.c_str());

    image_transport::ImageTransport it(nh);

    left_sub_.subscribe(it, left_topic, 1);
    right_sub_.subscribe(it, right_topic, 1);


   // Synchronize input topics. Optionally do approximate synchronization.
    local_nh.param("queue_size", queue_size_, 20);
    bool approx;
    local_nh.param("approximate_sync", approx, false);


    if (approx)
    {
      approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_),left_sub_,  right_sub_) );
      approximate_sync_->registerCallback(boost::bind(&sync_callback, _1, _2));
    }
    else
    {
      exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_),left_sub_, right_sub_) );
      exact_sync_->registerCallback(boost::bind(&sync_callback, _1, _2));
    }

	ros::spin();
	return 0;
}





