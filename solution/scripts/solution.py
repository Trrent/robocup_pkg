#!/usr/bin/env python3
import sys

import rospy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


import numpy as np
from math import cos, sin, pi

l = [0.033, 0.155, 0.3524]
alpha = [pi / 2, 0, 0]
d = [0.147, 0, 0]
q = [pi * 169.0 / 180.0, pi * 65.0 / 180.0 + pi / 2, -pi * 146.0 / 180.0]


def get_transform_matrix(a, alpha, d, theta):
    return np.array([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
                     [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
                     [0, sin(alpha), cos(alpha), d],
                     [0, 0, 0, 1]])


def find_jacob_matrix(q):
    positions = np.array([np.eye(4), np.eye(4), np.eye(4), np.eye(4)])
    for i in range(1, 4):
        for b in range(i, 4):
            positions[b] = positions[b] @ get_transform_matrix(l[i - 1], alpha[i - 1], d[i - 1], q[i - 1])
    Os = []
    Zpos = []
    for i in range(4):
        pos = positions[i]
        Zpos.append(pos[:3, 2])
        Os.append(pos[:3, 3])
    Jv = []
    Jw = []
    for i in range(1, 4):
        Jv.append(np.cross(np.array(Zpos[i - 1]), np.array(Os[3]) - np.array(Os[i - 1])))
        Jw.append(Zpos[i - 1])
    return np.transpose(Jv)


def find_pos(q):
    t = np.eye(4)
    for i in range(3):
        t = t @ get_transform_matrix(l[i], alpha[i], d[i], q[i])
    pos = t[:3, 3]
    return pos


def get_angles(x, y, z):
    global q
    prev_coord = np.array(find_pos(q))
    next_coord = np.array([x, y, z])
    for i in range(10):
        q = q + np.linalg.pinv(find_jacob_matrix(q)) @ (next_coord - prev_coord)
    q = [pi * q[0] / 180.0, pi * q[1] / 180.0, pi * q[2] / 180.0, 0, 0]
    return q


def make_trajectory_msg(joint_trajectory_plan=[], seq=0, secs=0, nsecs=0, dt=2, frame_id='/base_link'):
    jt = JointTrajectory()
    jt.header.seq = seq
    jt.header.stamp.secs = 0  # secs
    jt.header.stamp.nsecs = 0  # nsecs
    jt.header.frame_id = frame_id
    jt.joint_names = [f"arm_joint_{i}" for i in range(1, 6)]
    njtp = 1
    point = JointTrajectoryPoint()
    point.positions = joint_trajectory_plan
    point.time_from_start = rospy.Duration(1)
    jt.points.append(point)
    return jt


def move_robot():
    rospy.init_node('solution', anonymous=False)

    gazebo_command_publisher = rospy.Publisher('/arm_1/arm_controller/command', JointTrajectory, queue_size=10)
    r = rospy.Rate(10)
    angles = get_angles(*[10, 0, 0])
    # angles = [0, 0, 0, 0, 0]
    jt = make_trajectory_msg(joint_trajectory_plan=angles, dt=0.2, frame_id='base_link')

    while not rospy.is_shutdown():
        gazebo_command_publisher.publish(jt)
        print(angles)
        r.sleep()



if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion.", file=sys.stderr)
