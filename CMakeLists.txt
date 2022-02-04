cmake_minimum_required(VERSION 3.0.2)
project(lidar2depth)


add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS CATKIN_DEPENDS roscpp rospy std_msgs tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar2depth src/lidar2depth.cpp)
target_link_libraries(lidar2depth 
  ${catkin_LIBRARIES}  #${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}
)
