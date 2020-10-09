#include "helper.h"
#include <opencv2/opencv.hpp>
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




// template <class Type, class Consumer>
// void distribute_serialized(const ::lcm::ReceiveBuffer* rbuf, const std::std::string& channel, const vulcan::serialized_t* data, Consumer* consumer)
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
    lcmInstance_->subscribe(imuSubscriberChannel_,&helper::handleIMU,this);
    lcmInstance_->subscribe(imageSubscriberChannel_,&helper::handleImage,this);
    imuPublisher_ = rosNh_->advertise<sensor_msgs::Imu>(imuPublisherTopic_,1); // buffer up 1 message
    imagePublisher_ = rosNh_->advertise<sensor_msgs::Image>(imagePublisherTopic_,1); // buffer up 1 message
    clockPublisher_ = rosNh_->advertise<rosgraph_msgs::Clock>("clock",1); // buffer up 1 message
}

void helper::handleIMU(const lcm::ReceiveBuffer *rbuf, const std::string& channel, 
         const mbot_imu_t* msg)
{
    sensor_msgs::Imu imuROS;
    convertIMULCMToROS(msg,imuROS);
}

void helper::handleImage(const lcm::ReceiveBuffer *rbuf, const std::string& channel, 
         const image_t* msg)
{
    sensor_msgs::Image imageROS;
    convertImageLCMToROS(msg,imageROS);
}

void helper::convertIMULCMToROS(const mbot_imu_t* imuLCM, sensor_msgs::Imu& imuROS)
{
    int64_t tstamp=imuLCM->utime;// in microsecs
    imuROS.header.stamp.sec=(uint32_t)(tstamp/1e6); // in secs
    double diff=(tstamp-imuROS.header.stamp.sec*1e6); // diff in microsecs
    imuROS.header.stamp.nsec=(uint32_t)diff*1e3; // in nanosecs
    imuROS.header.frame_id="imu_link";
    Quaternion q=ToQuaternion(imuLCM->tb_angles[1],imuLCM->tb_angles[0],imuLCM->tb_angles[2]);
    imuROS.orientation.x=q.x;
    imuROS.orientation.y=q.y;
    imuROS.orientation.z=q.z;
    imuROS.orientation.w=q.w;
    imuROS.angular_velocity.x=imuLCM->gyro[0];
    imuROS.angular_velocity.y=imuLCM->gyro[1];
    imuROS.angular_velocity.z=imuLCM->gyro[2];
    imuROS.linear_acceleration.x=imuLCM->accel[0];  
    imuROS.linear_acceleration.y=imuLCM->accel[1];
    imuROS.linear_acceleration.z=imuLCM->accel[2];
    imuPublisher_.publish(imuROS);
    std::cout << "published imu as ros message" << '\n';
    // rosgraph_msgs::Clock clock;
    // ros::Time rosTimeInSec(imuROS.header.stamp.sec);
    // clock.clock=rosTimeInSec;
    // clockPublisher_.publish(clock);
}

void helper::convertImageLCMToROS(const image_t* imageLCM, sensor_msgs::Image& imageROS)
{
    int64_t tstamp=imageLCM->utime;// in microsecs
    imageROS.header.stamp.sec=(uint32_t)(tstamp/1e6); // in secs
    double diff=(tstamp-imageROS.header.stamp.sec*1e6); // diff in microsecs
    imageROS.header.stamp.nsec=(uint32_t)diff*1e3; // in nanosecs
    imageROS.header.frame_id="camera_link";
    imageROS.width=imageLCM->width;
    imageROS.height=imageLCM->height;
    imageROS.is_bigendian=0;
    cv::Mat imgDecode;
    imgDecode = cv::imdecode(imageLCM->data, cv::IMREAD_COLOR);
    imageROS.encoding="bgr8";
    imageROS.step=imgDecode.step;
    // https://stackoverflow.com/questions/26681713/convert-mat-to-array-vector-in-opencv
    imageROS.data.assign(imgDecode.data,imgDecode.data+imgDecode.total()*imgDecode.channels());
    imagePublisher_.publish(imageROS);
    std::cout << "published image as ros message" << '\n';
}


bool helper::readParameters()
{
    if(!rosNh_->getParam("imu_subscriber_channel",imuSubscriberChannel_)) return false;
    if(!rosNh_->getParam("imu_publisher_topic",imuPublisherTopic_)) return false;
    if(!rosNh_->getParam("image_subscriber_channel",imageSubscriberChannel_)) return false;
    if(!rosNh_->getParam("image_publisher_topic",imagePublisherTopic_)) return false;
    return true;
}