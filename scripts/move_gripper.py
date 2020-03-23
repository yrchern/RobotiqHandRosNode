#!/usr/bin/env python

import rospy
import sys
from RobotiqHandRosNode.msg import GripperControl


def check_bound(var):
    if var > 255:
        var = 255
    elif var < 0:
        var = 0
    return var


def set_gripper(pub, position, speed, force):
    rospy.loginfo("Setting gripper: position=" + str(position) +
                  " speed=" + str(speed) + " force=" + str(force))
    control_message = GripperControl()
    position = check_bound(position)
    speed = check_bound(speed)
    force = check_bound(force)
    control_message.position = position
    control_message.speed = speed
    control_message.force = force
    pub.publish(control_message)
    rospy.sleep(1.0)
    return


def main():

    # create a publisher
    pub = rospy.Publisher('RobotiqHandGripperControl', GripperControl, queue_size=10)
    # add a short delay before publisher is ready
    rospy.sleep(0.5)

    # move the gripper
    try:
        position = int(sys.argv[1])
        speed = int(sys.argv[2])
        force = int(sys.argv[3])
        set_gripper(pub, position, speed, force)
    except Exception as e:
        rospy.loginfo("Error: " + str(e))


if __name__ == '__main__':
    rospy.init_node("set_gripper", anonymous=False)
    if len(sys.argv) < 4:
        rospy.loginfo("Usage: rosrun RobotiqHandRosNode move_gripper.py [position] [speed] [force]")
    else:
        main()
