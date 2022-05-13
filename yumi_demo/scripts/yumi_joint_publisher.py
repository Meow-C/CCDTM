#!/usr/bin/env python
import numpy
import rospy
import tf
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_matrix


def yumi_joint_publisher():
    rospy.init_node('yumi_joint_publisher', anonymous=True)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    tf_listener = tf.TransformListener()

    rate = rospy.Rate(10)  # 10hz

    left_joint_name = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l',
                       'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l',
                       'yurmi_joint_6_l', 'gripper_l_joint', 'gripper_l_joint_m']

    right_joint_name = ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r',
                        'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r',
                        'yumi_joint_6_r', 'gripper_r_joint', 'gripper_r_joint_m']

    index = 0
    left_link_name = ['yumi_body', 'yumi_link_1_l', 'yumi_link_2_l', 'yumi_link_3_l',
                      'yumi_link_4_l', 'yumi_link_5_l', 'yumi_link_6_l',
                      'yumi_link_7_l', 'gripper_l_base', 'gripper_l_finger_l']

    right_link_name = ['yumi_body', 'yumi_link_1_r', 'yumi_link_2_r', 'yumi_link_3_r',
                       'yumi_link_4_r', 'yumi_link_5_r', 'yumi_link_6_r',
                       'yumi_link_7_r', 'gripper_r_base', 'gripper_r_finger_l']

    file = open('raw_data_r.txt', 'w')

    while not rospy.is_shutdown():

        left_joint_position = [0, 0, 0, 0, 0, 0,
                               0, 0, 0]
        right_joint_position = [0, 0, 0, 0, 0, 0,
                                0, 0, 0]

        joints = JointState()
        joints.header.stamp = rospy.Time.now()
        joints.name = left_joint_name + right_joint_name
        joints.position = left_joint_position + right_joint_position

        # pub.publish(joints)



        # parent_frame = left_link_name[index]
        # if index == len(left_link_name) - 1:
        #     file.close()
        #     break
        # child_frame = left_link_name[index + 1]

        parent_frame = right_link_name[index]
        if index == len(right_link_name) - 1:
            file.close()
            break
        child_frame = right_link_name[index + 1]

        try:
            position, quaternion = tf_listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(quaternion)
            matrix = quaternion_matrix(quaternion)

            file.writelines('Transform from ' + parent_frame + ' to ' + child_frame + ':\n')
            file.writelines('position :')
            file.writelines(str(position))
            file.write('\n')
            file.writelines('quaternion :')
            file.writelines(str(quaternion))
            file.write('\n')

            print '\nTransform from\t', parent_frame, ' to\t', child_frame, \
                '\nTranslation:', position, \
                '\nRotation:', quaternion

            index += 1

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue
        rate.sleep()


if __name__ == '__main__':
    try:
        yumi_joint_publisher()
    except rospy.ROSInterruptException:
        pass
