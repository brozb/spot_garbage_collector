cmake_minimum_required(VERSION 3.0.2)
project(spot_garbage_collector)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_py
  tf2_sensor_msgs
  ros_numpy
  depth_image_proc
  std_srvs
  spot_msgs
  pcl_ros
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_service_files(
  FILES
  MultiGrasp.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs   sensor_msgs   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spot_garbage_collector
#  CATKIN_DEPENDS rospy std_msgs
  DEPENDS Eigen
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(lidar_filter src/lidar_filter.cpp)
target_link_libraries(lidar_filter ${catkin_LIBRARIES})
