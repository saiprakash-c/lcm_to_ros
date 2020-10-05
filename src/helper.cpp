#include "helper.h"
// #include <boost/iostreams/device/array.hpp>
// #include <boost/iostreams/device/back_inserter.hpp>
// #include <boost/iostreams/stream.hpp>
// #include <cereal/cereal.hpp>
// #include <cereal/archives/binary.hpp>
// #include "serialized_t.hpp"


struct Quaternion
{
    double w, x, y, z;
};

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


void handleData(const lcm_recv_buf_t *rbuf, const char * channel, 
         const vulcan_lcm_imu_t * msg, void * user)
{
    helper* h=(helper*)user;
    sensor_msgs::Imu imuROS;
    h->convertLCMToROS(msg,imuROS);
   
}

// template <class Type, class Consumer>
// void distribute_serialized(const ::lcm::ReceiveBuffer* rbuf, const std::string& channel, const vulcan::serialized_t* data, Consumer* consumer)
// {
//     try
//     {
//         boost::iostreams::array_source inputBuf(data->data.data(), data->data.size());
//         boost::iostreams::stream<decltype(inputBuf)> input(inputBuf);
//         cereal::BinaryInputArchive in(input);

//         Type t;
//         in >> t;

//         consumer->handleData(t, channel);
//     }
//     catch(std::exception& e)
//     {
//         std::cerr << "EXCEPTION: Failed to unserialize message of type: " << typeid(Type).name() << " Error : " << e.what() << " Message discarded!\n";
//     }
// }

helper::helper(lcm::LCM* l, ros::NodeHandle* nh):
lcmInstance_(l),rosNh_(nh)
{
    if(!readParameters())
    {
        std::cout << "Parameters could not be loaded!" << std::endl;
    }
    lcm_t* lcmPtr = lcmInstance_->getUnderlyingLCM();
    vulcan_lcm_imu_t_subscribe(lcmPtr,subscriberTopic_.c_str(),&handleData, this);
    imuPublisher_ = rosNh_->advertise<sensor_msgs::Imu>(publisherTopic_,1); // buffer up 1 message
    clockPublisher_ = rosNh_->advertise<rosgraph_msgs::Clock>("clock",1); // buffer up 1 message
}



void helper::convertLCMToROS(const vulcan_lcm_imu_t* imuLCM, sensor_msgs::Imu& imuROS)
{
    int64_t tstamp=imuLCM->timestamp;// in microsecs
    imuROS.header.stamp.sec=(uint32_t)(tstamp/1e6); // in secs
    double diff=(tstamp-imuROS.header.stamp.sec*1e6); // diff in microsecs
    imuROS.header.stamp.nsec=(uint32_t)diff*1e3; // in nanosecs
    imuROS.header.frame_id="imu_link";
    Quaternion q=ToQuaternion(imuLCM->orientation[0],imuLCM->orientation[1],imuLCM->orientation[2]);
    imuROS.orientation.x=q.x;
    imuROS.orientation.y=q.y;
    imuROS.orientation.z=q.z;
    imuROS.orientation.w=q.w;
    imuROS.angular_velocity.x=imuLCM->rotational_vel[2];
    imuROS.angular_velocity.y=imuLCM->rotational_vel[1];
    imuROS.angular_velocity.z=imuLCM->rotational_vel[0];
    imuROS.linear_acceleration.x=imuLCM->accel[2];
    imuROS.linear_acceleration.y=imuLCM->accel[1];
    imuROS.linear_acceleration.z=imuLCM->accel[0];
    imuPublisher_.publish(imuROS);
    // rosgraph_msgs::Clock clock;
    // ros::Time rosTimeInSec(imuROS.header.stamp.sec);
    // clock.clock=rosTimeInSec;
    // clockPublisher_.publish(clock);
}

bool helper::readParameters()
{
    if(!rosNh_->getParam("subscriber_channel",subscriberTopic_)) return false;
    if(!rosNh_->getParam("publisher_topic",publisherTopic_)) return false;
    return true;
}