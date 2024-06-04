#!/usr/bin/env python3
import sys
import rospy
from math import radians

from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from sensor_msgs.msg import JointState


from solution import find_pos, get_angles, make_trajectory_msg

class ManipulatorManager:
    def __init__(self):
        self.pub = rospy.Publisher('/arm_1/arm_controller/command', JointTrajectory, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.update_position)

        self.position = []

    def update_position(self, data):
        self.position = data.position[:5]

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            coord = [float(i) for i in input().split()]
            self.position = get_angles(self.position[:3], coord[0], coord[1], coord[2])
            jt = make_trajectory_msg(joint_trajectory_plan=self.position)
            self.pub.publish(jt)
            rate.sleep()


def main():
    rospy.init_node('solution', anonymous=False)

    manager = ManipulatorManager()
    manager.spin()

if __name__ == "__main__":
    main()
