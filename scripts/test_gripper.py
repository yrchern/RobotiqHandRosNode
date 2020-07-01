#!/usr/bin/env python

import rospy
from robotiq_hand_ros_node.msg import GripperControl


def set_gripper(pub, position, speed, force):
    rospy.loginfo("Setting gripper: position=" + str(position) +
                  " speed=" + str(speed) + " force=" + str(force))
    control_message = GripperControl()
    control_message.position = position
    control_message.speed = speed
    control_message.force = force
    pub.publish(control_message)
    rospy.sleep(1.0)
    return


def main():
    rospy.init_node("set_gripper", anonymous=False)
    # create a publisher
    pub = rospy.Publisher('RobotiqHandGripperControl',
                          GripperControl, queue_size=10)
    # add a short delay before publisher is ready
    rospy.sleep(0.5)

    # close gripper
    rospy.loginfo("Closing gripper")
    set_gripper(pub, 255, 255, 1)
    rospy.sleep(5.0)

    # open gripper
    rospy.loginfo("Opening gripper")
    set_gripper(pub, 0, 255, 0)
    rospy.sleep(1.0)


if __name__ == '__main__':
    main()
