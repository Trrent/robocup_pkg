#!/usr/bin/env python3
import sys
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from sensor_msgs.msg import JointState
import numpy as np
from math import cos, sin, pi, radians

# Parameters
l = [0.033, 0.155, 0.3524]
alpha = [pi / 2, 0, 0]
d = [0.147, 0, 0]
q = [0, 0, 0]
q_deviation = np.array([pi * 169.0 / 180.0, pi * 65.0 / 180.0 + pi / 2, -pi * 146.0 / 180.0, pi * 102.5 / 180.0, pi * 167.5 / 180.0])


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


def get_angles(prev_angles, x, y, z):
    prev_coord = np.array(find_pos(prev_angles[:3]))
    next_coord = np.array([x, y, z])
    for i in range(10):
        prev_angles = prev_angles + np.linalg.pinv(find_jacob_matrix(prev_angles)) @ (next_coord - prev_coord)
        prev_coord = np.array(find_pos(prev_angles))
    prev_angles = np.append([i % (2 * pi) for i in prev_angles], [radians(102.5), radians(167.5)])
    # q = [-round(i, 2) for i in prev_angles]
    # prev_angles = np.array([q[0] + q_deviation[0], q[1] + q_deviation[1], q[2] + q_deviation[2], q_deviation[3], q_deviation[4]])
    return prev_angles


def make_trajectory_msg(joint_trajectory_plan):
    jt = JointTrajectory()
    jt.header.seq = 0
    jt.header.stamp.secs = 0  # secs
    jt.header.stamp.nsecs = 0  # nsecs
    jt.header.frame_id = '/base_link'
    jt.joint_names = [f"arm_joint_{i}" for i in range(1, 6)]
    point = JointTrajectoryPoint()
    point.positions = joint_trajectory_plan
    point.time_from_start = rospy.Duration(1)
    jt.points.append(point)
    return jt


def move_robot():
    global q
    rospy.init_node('solution', anonymous=False)

    gazebo_command_publisher = rospy.Publisher('/arm_1/arm_controller/command', JointTrajectory, queue_size=10)
    rospy.Subscriber('/joint_states', JointState, update_position)
    r = rospy.Rate(10)

    # coord = find_pos([0, pi / 2, 0]) # x, y, z
    # q, angles = get_angles(q, coord[0], coord[1], coord[2])
    # print(q)
    # print(angles)
    #
    # jt = make_trajectory_msg(joint_trajectory_plan=angles)
    # gazebo_command_publisher.publish(jt)

    while not rospy.is_shutdown():
        try:
            coord = find_pos([pi * int(i) / 180 for i in input().split()])
            # coord = [float(i) for i in input().split()]
            q, angles = get_angles(q, coord[0], coord[1], coord[2])
            print(q)
            print(angles)
            jt = make_trajectory_msg(joint_trajectory_plan=angles)
            gazebo_command_publisher.publish(jt)
            r.sleep()
        except Exception as e:
            print(e)
            pass


if __name__ == '__main__':
    move_robot()
