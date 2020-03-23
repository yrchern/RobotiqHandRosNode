#!/usr/bin/env python

# A Simple Gripper Node based on RobotiqHand https://github.com/TechMagicKK/RobotiqHand


import rospy
import sys

from RobotiqHand import RobotiqHand

from RobotiqHandRosNode.msg import GripperControl
from RobotiqHandRosNode.msg import GripperMoveResult


def signal_handler(sig, frame):
    rospy.loginfo('User pressed Ctrl+C to terminate the program')
    sys.exit(0)


class RobotiqHandNode():
    def __init__(self, addr, port):
        self.addr = addr
        self.port = port
        self.hand = RobotiqHand()
        rospy.loginfo("Waiting for 30 seconds for ros_control to start on the robot")
        rospy.sleep(30.0)
        self.hand.connect(self.addr, int(self.port))
        self.hand.reset()
        self.hand.activate()
        result = self.hand.wait_activate_complete()
        rospy.loginfo('activate: result = 0x{:02x}'.format(result))
        if result != 0x31:
            self.hand.disconnect()
            sys.exit(1)
            return
        rospy.Subscriber('RobotiqHandGripperControl',
                         GripperControl, self.controller)
        self.pub = rospy.Publisher('RobotiqHandGripperMoveResult', GripperMoveResult, queue_size=10)
        rospy.on_shutdown(self.cleanup)

    def controller(self, msg):
        rospy.loginfo('Command received: position =' + str(msg.position) +
                      ' speed = ' + str(msg.speed) + ', force = ' + str(msg.force))
        self.hand.move(msg.position, msg.speed, msg.force)
        (status, position, force) = self.hand.wait_move_complete()
        position_mm = self.hand.get_position_mm(position)
        force_mA = self.hand.get_force_mA(force)
        moveResult = GripperMoveResult()
        moveResult.status = status
        moveResult.position = position
        moveResult.force = force
        self.pub.publish(moveResult)
        rospy.loginfo('status =' + str(status) +
                      ' position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA))

    def cleanup(self):
        rospy.loginfo("Disconnecting...")
        self.hand.disconnect()
        return


if __name__ == '__main__':
    rospy.init_node('RobotiqHand')
    # there might be some args if launching from a launch file
    if len(sys.argv) < 3:
        rospy.logerr("Usage: rosrun robotiq_hand_node robotiq_hand_rosnode.py [IP Addr] [Port]")

    else:
        try:
            # TODO: Verify the IP address and port number
            rospy.loginfo("Connecting to the gripper...")
            gripper = RobotiqHandNode(sys.argv[1], sys.argv[2])
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
