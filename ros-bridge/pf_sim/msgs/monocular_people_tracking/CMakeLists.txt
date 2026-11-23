cmake_minimum_required(VERSION 3.0.2)
project(monocular_people_tracking)

find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs geometry_msgs tfpose_ros)

add_message_files(FILES Track.msg TrackArray.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs tfpose_ros)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs tfpose_ros)
