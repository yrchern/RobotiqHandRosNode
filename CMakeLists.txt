cmake_minimum_required(VERSION 2.8.3)
project(robotiq_hand_ros_node)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  GripperControl.msg
  GripperMoveResult.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

