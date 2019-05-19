cmake_minimum_required(VERSION 2.8.3)
project(icpslam)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    pcl_conversions
    pcl_ros
    pose_graph_utils
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2
    tf2_ros
)

catkin_package(
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    pcl_conversions
    pcl_ros
    pose_graph_utils
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2
    tf2_ros
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

## Build utils library
add_library(ICPSLAM_UTILS 
  src/utils/pose6DOF.cpp
  src/utils/geometric_utils.cpp
  src/utils/messaging_utils.cpp
)
target_link_libraries(ICPSLAM_UTILS
  ${catkin_LIBRARIES}
)


## Build library from icpslam source files
add_library(${PROJECT_NAME}
  src/icpslam/robot_odometer.cpp
  src/icpslam/icp_odometer.cpp
  src/icpslam/octree_mapper.cpp
  src/icpslam/icpslam.cpp
)
target_link_libraries(${PROJECT_NAME}
  ICPSLAM_UTILS
)


## ICP SLAM
add_executable(icpslam_node
  src/icpslam_node.cpp
)
target_link_libraries(icpslam_node
  ${PROJECT_NAME}
)