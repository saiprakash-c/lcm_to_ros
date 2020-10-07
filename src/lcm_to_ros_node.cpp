#include <lcm/lcm-cpp.hpp>
#include <ros/ros.h>
#include "helper.h"

int main(int argc, char* argv[])
{

    // initialize lcm
    lcm::LCM lcmObject;
     if(!lcmObject.good())
        return 1;

    // initialize ros
    ros::init(argc,argv,"lcm_to_ros");
    ros::NodeHandle h("~");
    helper publishImu(&lcmObject,&h);

    while(true)
    {
        lcmObject.handle();
    }
    return true;
}