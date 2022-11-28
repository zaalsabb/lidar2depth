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
  geometry_msgs
  std_msgs
  tf2_ros
  message_generation
)

 add_message_files(
   FILES
   Mesh.msg
   Submap.msg
 )

 add_service_files(
  FILES
  GetMap.srv
  GetPoses.srv
  GetStamps.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS CATKIN_DEPENDS roscpp rospy std_msgs tf2_ros
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar2depth src/lidar2depth.cpp)
add_executable(lidar2mesh src/lidar2mesh.cpp)
add_executable(map2color src/map2color.cpp)
add_executable(lidar2submap src/lidar2submap.cpp)
add_executable(submap2lidar src/submap2lidar.cpp)
add_executable(bag2depth src/bag2depth.cpp)

target_link_libraries(lidar2depth 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} #${PCL_LIBRARIES}  ${CERES_LIBRARIES}
)

target_link_libraries(lidar2mesh 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} #${PCL_LIBRARIES}  ${CERES_LIBRARIES}
)

target_link_libraries(map2color 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} #${PCL_LIBRARIES}  ${CERES_LIBRARIES}
)

target_link_libraries(lidar2submap 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} #${PCL_LIBRARIES}  ${CERES_LIBRARIES}
)

target_link_libraries(submap2lidar 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} #${PCL_LIBRARIES}  ${CERES_LIBRARIES}
)

add_dependencies(lidar2mesh lidar2depth_generate_messages_cpp)

target_link_libraries(bag2depth 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} #${PCL_LIBRARIES}  ${CERES_LIBRARIES}
)