#ifndef STEREO_PROCESSOR_H_
#define STEREO_PROCESSOR_H_


#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

class StereoProcessor
{

private:

  // subscriber
  image_transport::SubscriberFilter left_sub_, right_sub_,  
                                    left_sub_ZBL, right_sub_ZBR,
                                    left_sub_ZFL, right_sub_ZFR,
                                    left_sub_ZFLL, right_sub_ZFLR,
                                    left_sub_ZFRL, right_sub_ZFRR,
                                    left_sub_ZBLL, right_sub_ZBLR,
                                    left_sub_ZBRL, right_sub_ZBRR;

  typedef message_filters::sync_policies::ExactTime<  sensor_msgs::Image, sensor_msgs::Image,
                                                      sensor_msgs::Image, sensor_msgs::Image,
                                                      sensor_msgs::Image, sensor_msgs::Image
                                                      > ExactPolicy;

  typedef message_filters::sync_policies::ApproximateTime<  sensor_msgs::Image, sensor_msgs::Image,
                                                            sensor_msgs::Image, sensor_msgs::Image,
                                                            sensor_msgs::Image, sensor_msgs::Image
                                                            > ApproximatePolicy;

  // typedef message_filters::sync_policies::ExactTime<  sensor_msgs::Image, sensor_msgs::Image,
  //                                                     sensor_msgs::Image, sensor_msgs::Image,
  //                                                     sensor_msgs::Image, sensor_msgs::Image,
  //                                                     sensor_msgs::NavSatFix, nav_msgs::Odometry> ExactPolicy;

  // typedef message_filters::sync_policies::ApproximateTime<  sensor_msgs::Image, sensor_msgs::Image,
  //                                                           sensor_msgs::Image, sensor_msgs::Image,
  //                                                           sensor_msgs::Image, sensor_msgs::Image,
  //                                                           sensor_msgs::NavSatFix, nav_msgs::Odometry> ApproximatePolicy;

  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  int queue_size_;

  // for sync checking
  ros::WallTimer check_synced_timer_;
  int left_received_, right_received_,
        ZBL,
        ZBR,
        ZFL,
        ZFR, all_received_;

  // for sync checking
  static void increment(int* value)
  {
    ++(*value);
  }

  // void dataCb(const sensor_msgs::ImageConstPtr& l_image_msg,
  //             const sensor_msgs::ImageConstPtr& r_image_msg,
  //             const sensor_msgs::ImageConstPtr& msg_ZBL,
  //             const sensor_msgs::ImageConstPtr& msg_ZBR,
  //             const sensor_msgs::ImageConstPtr& msg_ZFL,
  //             const sensor_msgs::ImageConstPtr& msg_ZFR,
  //             const sensor_msgs::NavSatFix::ConstPtr& msg_fix,
  //             const nav_msgs::Odometry::ConstPtr& msg_odom)
  // {

  //   // For sync error checking
  //   ++all_received_;

  //   // call implementation
  //   imageCallback(l_image_msg, r_image_msg,
  //                 msg_ZBL,
  //                 msg_ZBR,
  //                 msg_ZFL,
  //                 msg_ZFR,
  //                 msg_fix,
  //                 msg_odom);
  // }


  void dataCb(const sensor_msgs::ImageConstPtr& l_image_msg,
              const sensor_msgs::ImageConstPtr& r_image_msg,
              const sensor_msgs::ImageConstPtr& msg_ZBL,
              const sensor_msgs::ImageConstPtr& msg_ZBR,
              const sensor_msgs::ImageConstPtr& msg_ZFL,
              const sensor_msgs::ImageConstPtr& msg_ZFR)
  {

    // For sync error checking
    ++all_received_;

    // call implementation
    imageCallback(l_image_msg, r_image_msg,
                  msg_ZBL,
                  msg_ZBR,
                  msg_ZFL,
                  msg_ZFR);
  }



  void checkInputsSynchronized()
  {
    int threshold = 3 * all_received_;
    if (left_received_ >= threshold || right_received_ >= threshold) {
      ROS_WARN("[stereo_processor] Low number of synchronized left/right tuples received.\n"
               "Left images received:       %d (topic '%s')\n"
               "Right images received:      %d (topic '%s')\n"
               "Synchronized tuples: %d\n"
               "Possible issues:\n"
               "\t* stereo_image_proc is not running.\n"
               "\t  Does `rosnode info %s` show any connections?\n"
               "\t* The cameras are not synchronized.\n"
               "\t  Try restarting the node with parameter _approximate_sync:=True\n"
               "\t* The network is too slow. One or more images are dropped from each tuple.\n"
               "\t  Try restarting the node, increasing parameter 'queue_size' (currently %d)",
               left_received_, left_sub_.getTopic().c_str(),
               right_received_, right_sub_.getTopic().c_str(),
               all_received_, ros::this_node::getName().c_str(), queue_size_);
    }
  }


protected:


  StereoProcessor(const std::string& transport) :
    left_received_(0), right_received_(0), all_received_(0)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");

    // Resolve topic names
    ros::NodeHandle nh;
    std::string stereo_ns = nh.resolveName("/stereo");
    std::string zed = nh.resolveName("/zed");
    std::string rect_color =  nh.resolveName("image_rect_color/");

    std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image_raw/"));
    std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh.resolveName("image_raw/"));

    // /zed/back/left/image_rect_color // /zed/back/right/image_rect_color
    std::string back_left = ros::names::clean(zed + "/back" + "/left/" + rect_color );
    std::string back_right = ros::names::clean(zed + "/back" + "/right/" + rect_color);

    // /zed/front/left/image_rect_color_low_freq /zed/front/right/image_rect_color_low_freq  
    std::string front_left = ros::names::clean(zed + "/front" + "/left/" + nh.resolveName("image_rect_color_low_freq/"));
    std::string front_right = ros::names::clean(zed + "/front" + "/right/" + nh.resolveName("image_rect_color_low_freq/"));

    std::string fix = ros::names::clean("/applanix/gps_fix");
    std::string odom = ros::names::clean("/applanix/odom");


    // // Subscribe to four input topics.
    // ROS_INFO("Subscribing to:\n\t* %s\n\t*  %s\n\t*  %s\n\t*  %s\n\t* %s\n\t*  %s\n\t* %s\n\t* ",
    //     left_topic.c_str(), right_topic.c_str(), 
    //     back_left.c_str(), back_right.c_str(),
    //     front_left.c_str(), front_right.c_str(),
    //     odom.c_str(), fix.c_str());


    image_transport::ImageTransport it(nh);

    left_sub_.subscribe(it, left_topic, 3, transport);
    right_sub_.subscribe(it, right_topic, 3, transport);

    left_sub_ZBL.subscribe(it, back_left, 3, transport);
    right_sub_ZBR.subscribe(it, back_right, 3, transport);

    left_sub_ZFL.subscribe(it, front_left, 3, transport);
    right_sub_ZFR.subscribe(it, front_right, 3, transport);

    // message_filters::Subscriber<sensor_msgs::NavSatFix> sub_fix(nh, fix, 3);
    // message_filters::Subscriber<nav_msgs::Odometry> sub_odom(nh, odom, 3);


    // Complain every 15s if the topics appear unsynchronized
    left_sub_.registerCallback(boost::bind(StereoProcessor::increment, &left_received_));
    right_sub_.registerCallback(boost::bind(StereoProcessor::increment, &right_received_));


    check_synced_timer_ = nh.createWallTimer(ros::WallDuration(15.0), boost::bind(&StereoProcessor::checkInputsSynchronized, this));

    // Synchronize input topics. Optionally do approximate synchronization.
    local_nh.param("queue_size", queue_size_, 5);
    bool approx;
    local_nh.param("approximate_sync", approx, false);



    if (approx)
    {
      approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_),left_sub_,
                                                                      right_sub_,
                                                                      left_sub_ZBL,
                                                                      right_sub_ZBR,
                                                                      left_sub_ZFL,
                                                                      right_sub_ZFR) );
      approximate_sync_->registerCallback(boost::bind(&StereoProcessor::dataCb, this, _1, _2, _3, _4, _5, _6));
    }
    else
    {
      exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_),left_sub_, 
                                                              right_sub_,
                                                              left_sub_ZBL,
                                                              right_sub_ZBR,
                                                              left_sub_ZFL,
                                                              right_sub_ZFR) );
      exact_sync_->registerCallback(boost::bind(&StereoProcessor::dataCb, this, _1, _2, _3, _4, _5, _6));
    }
  }


virtual void imageCallback( const sensor_msgs::ImageConstPtr& l_image_msg,
                            const sensor_msgs::ImageConstPtr& r_image_msg,                          
                            const sensor_msgs::ImageConstPtr& msg_ZBL,
                            const sensor_msgs::ImageConstPtr& msg_ZBR,
                            const sensor_msgs::ImageConstPtr& msg_ZFL,
                            const sensor_msgs::ImageConstPtr& msg_ZFR) = 0;

};

  

//     if (approx)
//     {
//       approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_),left_sub_,
//                                                                       right_sub_,
//                                                                       left_sub_ZBL,
//                                                                       right_sub_ZBR,
//                                                                       left_sub_ZFL,
//                                                                       right_sub_ZFR, sub_fix, sub_odom) );
//       approximate_sync_->registerCallback(boost::bind(&StereoProcessor::dataCb, this, _1, _2, _3, _4, _5, _6, _7, _8));
//     }
//     else
//     {
//       exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_),left_sub_, 
//                                                               right_sub_,
//                                                               left_sub_ZBL,
//                                                               right_sub_ZBR,
//                                                               left_sub_ZFL,
//                                                               right_sub_ZFR, sub_fix, sub_odom) );
//       exact_sync_->registerCallback(boost::bind(&StereoProcessor::dataCb, this, _1, _2, _3, _4, _5, _6, _7, _8));
//     }
//   }


// virtual void imageCallback( const sensor_msgs::ImageConstPtr& l_image_msg,
//                             const sensor_msgs::ImageConstPtr& r_image_msg,                          
//                             const sensor_msgs::ImageConstPtr& msg_ZBL,
//                             const sensor_msgs::ImageConstPtr& msg_ZBR,
//                             const sensor_msgs::ImageConstPtr& msg_ZFL,
//                             const sensor_msgs::ImageConstPtr& msg_ZFR,
//                             const sensor_msgs::NavSatFix::ConstPtr& msg_fix,
//                             const nav_msgs::Odometry::ConstPtr& msg_odom) = 0;

// };

#endif

