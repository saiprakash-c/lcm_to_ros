cmake_minimum_required(VERSION 2.8.3)
project(imu_lcm_to_ros)

# compiler flags
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")


# find packages - catkin, boost, cereal
find_package(catkin REQUIRED COMPONENTS sensor_msgs roscpp)
find_package(Boost REQUIRED COMPONENTS iostreams)
# find_package(cereal REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    thirdparty
#  LIBRARIES
  CATKIN_DEPENDS 
    roscpp
    sensor_msgs
 DEPENDS 
    Boost
)
# include directories 
include_directories(include lcmtypes thirdparty/cereal/include src ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

# dependencies
set(DEPENDENCIES ${catkin_LIBRARIES} ${boost_LIBRARIES} lcm)

# install executables
add_executable(imu_lcm_to_ros src/imu_lcm_to_ros_node.cpp src/helper.cpp lcmtypes/vulcan_lcm_imu_t.c)
target_link_libraries(imu_lcm_to_ros ${DEPENDENCIES})