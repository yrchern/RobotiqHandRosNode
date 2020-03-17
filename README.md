# RobotiqHandRosNode

This is a ROS node to control Robotiq Gripper based on RobotiqHand (https://github.com/TechMagicKK/RobotiqHand).

## Requirements ##

In order to use RobotiqHandRosNode, you need to install [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) and setup [tool communication](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/setup_tool_communication.md). In addition, Robotiq Gripper URCap needs to be uninstalled on the robot (Please follow [RobotiqHand](https://github.com/TechMagicKK/RobotiqHand) for the instructions). 

## Installation ##

1. Clone this repository into `src/` of your ROS catkin workspace.
2. Copy `RobotiqHand.py` from [RobotiqHand](https://github.com/TechMagicKK/RobotiqHand) into `scripts/` subfolder of this package (`RobotiqHandRosNode/scripts/`).
3. In your ROS catkin workspace, run `catkin_make install`. This will generate the message files used in ROS topics.

## Usage ##

1. To start the node: `roslaunch RobotiqHandRosNode RobotiqHand_ROS_Node.py [ip address of your UR robot] 54321`
2. To control the gripper, simply publish messages to `RobotiqHandGripperControl` topic, check `test_gripper.py` for examples.



