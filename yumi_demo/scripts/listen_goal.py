#!/usr/bin/env python

import numpy
import math
import rospy
import tf
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_matrix
import std_msgs.msg

import roslib
roslib.load_manifest('joint_states_listener')
import rospy
from joint_states_listener.srv import *
from sensor_msgs.msg import JointState
import threading


def yumi_joint_publisher():
    rospy.init_node('yumi_state_publisher', anonymous=True)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    dist_pub = rospy.Publisher('/distance', std_msgs.msg.Float64, queue_size=10)
    tf_listener = tf.TransformListener()

    rate = rospy.Rate(10)  # 10hz

    left_joint_name = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l',
                       'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l',
                       'yumi_joint_6_l', 'gripper_l_joint', 'gripper_l_joint_m']

    right_joint_name = ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r',
                        'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r',
                        'yumi_joint_6_r', 'gripper_r_joint', 'gripper_r_joint_m']

    left_link_name = ['yumi_body', 'yumi_link_1_l', 'yumi_link_2_l', 'yumi_link_3_l',
                      'yumi_link_4_l', 'yumi_link_5_l', 'yumi_link_6_l',
                      'yumi_link_7_l', 'gripper_l_base', 'gripper_l_finger_l']

    right_link_name = ['yumi_body', 'yumi_link_1_r', 'yumi_link_2_r', 'yumi_link_3_r',
                       'yumi_link_4_r', 'yumi_link_5_r', 'yumi_link_6_r',
                       'yumi_link_7_r', 'gripper_r_base', 'gripper_r_finger_l']

    theta = 0

    while not rospy.is_shutdown():

        if theta < 1.57:
            theta += 0.01
        else:
            theta = -1.57

        left_joint_position = [theta, theta, theta, theta, theta, theta,
                               theta, 0, 0]
        right_joint_position = [theta, theta, theta, theta, theta, theta,
                                theta, 0, 0]

        joints = JointState()
        joints.header.stamp = rospy.Time.now()
        joints.name = left_joint_name + right_joint_name
        joints.position = left_joint_position + right_joint_position

        pub.publish(joints)

        try:
            position, quaternion = tf_listener.lookupTransform('gripper_l_finger_l', 'gripper_r_finger_l', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(quaternion)
            matrix = quaternion_matrix(quaternion)
            distance = math.sqrt(position[0] * position[0] + position[1] * position[1] + position[2] * position[2])
            dist_pub.publish(distance)

            print euler, '\n', matrix, '\n', distance, '\n'

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue
        rate.sleep()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.name)

def yumi_state_listener():
    rospy.init_node('yumi_state_listener', anonymous=True)

    rospy.Subscriber("/move_group/goal", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    yumi_state_listener()
