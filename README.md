# lcm_to_ros

This program converts LCM messgaes to ROS messages. Currently, it supports only image and imu messages. 

## Prerequisites

Before you begin, ensure you have met the following requirements:
<!--- These are just example requirements. Add, duplicate or remove as required --->

* You have a `Linux` machine. 
* [lcm 1.4.0](https://github.com/lcm-proj/lcm)
* [ros-melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
  - catkin is included by default when ros is installed 
* [OpenCV]


## Installing

To install the project, follow these steps:

```
cd ~/catkin_ws/src # go to src directory of your catkin work space
git clone https://github.com/saiprakash-c/lcm_to_ros.git
cd ../ # go to your catkin workspace
catkin build lcm_to_ros.launch
source devel/setup.bash
```
## Usage

To use lcm_to_ros to convert a lcm log file to a ros bag file, follow these steps:

1. Set the lcm channel names and ros topic names in config/config.yaml
2. Launch the lcm_to_ros.launch file as follows
```
cd ~/catkin_ws # go to your catkin workspace
roslaunch lcm_to_ros lcm_to_ros.launch 
```
3. Record ros messages to a bag file 
```
rosbag record -O <ros_bag_file_name>
```
4. Run your lcm log file
```
lcm-logplayer-gui <lcm_log_file>
```

## Contact

You can reach me at saip@umich.edu
