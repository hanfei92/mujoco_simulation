cmake_minimum_required(VERSION 3.0.2)
project(monocular_person_following)

find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs geometry_msgs)

add_message_files(FILES Target.msg RobotTarget.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
