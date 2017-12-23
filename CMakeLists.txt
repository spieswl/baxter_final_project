cmake_minimum_required(VERSION 2.8.3)
project(container-manipulator_baxter)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  baxter_interface
  ar_track_alvar
  )

add_service_files(
  FILES
  OffsetMove.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  ar_track_alvar_msgs
)

catkin_package()

include_directories()
