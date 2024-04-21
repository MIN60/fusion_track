cmake_minimum_required(VERSION 3.0.2)
project(fusion_track)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  vision_msgs
  message_filters
  cv_bridge
  tf
  tf_conversions
  std_msgs
  cam_lidar_calib
  morai_msgs
  message_generation
)



find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


add_message_files(
    FILES
    TrackCenter.msg
    IsCurve.msg
    ReadyToCurve.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  vision_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs cam_lidar_calib morai_msgs
  DEPENDS PCL
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(fusion_track_node src/main.cpp src/fusion_track.cpp src/visual_marker.cpp src/VehicleControl.cpp)

#add_executable(fusion_track_node src/main.cpp src/fusion_track.cpp)
target_link_libraries(fusion_track_node ${catkin_LIBRARIES})

add_library(pid_controller_lib src/pidController.cpp)
add_library(pure_pursuit_lib src/PPController.cpp)


target_link_libraries(fusion_track_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
   pid_controller_lib pure_pursuit_lib
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)