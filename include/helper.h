#ifndef HELPER_H
#define HELPER_H

#include <lcm/lcm-cpp.hpp>
#include "vulcan_lcm_imu_t.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <rosgraph_msgs/Clock.h>


class helper
{
    public:
        helper(lcm::LCM* lcmInstance, ros::NodeHandle* imuPublisher);
        // void handleData(const lcm_recv_buf_t *rbuf, const char * channel, 
        //  const vulcan_lcm_imu_t * msg, void * user);
        void convertLCMToROS(const vulcan_lcm_imu_t* imuLCM, sensor_msgs::Imu& imuROS);
    private: 
        
        bool readParameters();
        
        lcm::LCM* lcmInstance_;
        ros::NodeHandle* rosNh_;
        ros::Publisher imuPublisher_;
        ros::Publisher clockPublisher_;
        std::string subscriberTopic_;
        std::string publisherTopic_;
};

#endif