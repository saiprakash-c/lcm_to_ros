#ifndef HELPER_H
#define HELPER_H

#include <lcm/lcm-cpp.hpp>
#include "image_t.hpp"
#include "mbot_imu_t.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <rosgraph_msgs/Clock.h>


class helper
{
    public:
        helper(lcm::LCM* lcmInstance, ros::NodeHandle* imuPublisher);
        // void handleData(const lcm_recv_buf_t *rbuf, const char * channel, 
        //  const vulcan_lcm_imu_t * msg, void * user);
       
         void handleIMU(const lcm::ReceiveBuffer *rbuf, const std::string& channel, 
         const mbot_imu_t* msg);
         void handleImage(const lcm::ReceiveBuffer *rbuf, const std::string& channel, 
         const image_t* msg);
    private: 
        
        bool readParameters();
        void convertIMULCMToROS(const mbot_imu_t* imuLCM, sensor_msgs::Imu& imuROS);
        void convertImageLCMToROS(const image_t* imageLCM, sensor_msgs::Image& imageROS);
        lcm::LCM* lcmInstance_;
        ros::NodeHandle* rosNh_;
        ros::Publisher imuPublisher_;
        ros::Publisher imagePublisher_;
        ros::Publisher clockPublisher_;
        std::string imuSubscriberChannel_;
        std::string imuPublisherTopic_;
        std::string imageSubscriberChannel_;
        std::string imagePublisherTopic_;
};

#endif