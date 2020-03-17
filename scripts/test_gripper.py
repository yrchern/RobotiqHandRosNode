#!/usr/bin/env python

import rospy
from RobotiqHandRosNode.msg import GripperControl


def test_gripper():
    # create a publisher
    pub = rospy.Publisher('RobotiqHandGripperControl', GripperControl, queue_size=10)
    # add a short delay before publisher is ready
    rospy.sleep(0.5)

    # close gripper
    rospy.loginfo("Closing gripper")
    control_message = GripperControl()
    control_message.position = 255
    control_message.speed = 255
    control_message.force = 1
    pub.publish(control_message)
    rospy.sleep(5.0)

    # open gripper
    rospy.loginfo("Opening gripper")
    control_message = GripperControl()
    control_message.position = 0
    control_message.speed = 255
    control_message.force = 0
    pub.publish(control_message)
    rospy.sleep(1.0)


if __name__ == '__main__':
    rospy.init_node("test_gripper", anonymous=False)
    test_gripper()
