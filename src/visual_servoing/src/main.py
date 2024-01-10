#! /usr/bin/env python3

# *******************************************************************
# Author: Sungbin Park
# Jan. 2024
# Copyright 2024, Sungbin Park, All rights reserved.
# *******************************************************************

import roslib; roslib.load_manifest('kinova_demo')
import rospy
import tf2_ros

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

import numpy
import math

""" Global variable """
prefix = 'j2s7s300_'
current_pose = [0.212322831154, -0.257197618484, 0.509646713734, 0.6471141576766968, 0.31610047817230225, 0.4218596816062927, 0.5507795214653015]   # home pose in mq


def cartesian_pose_client(position_, orientation_):
    """Send a cartesian goal to the action server."""
    action_address = '/' + prefix + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position_[0], y=position_[1], z=position_[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation_[0], y=orientation_[1], z=orientation_[2], w=orientation_[3])

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(1)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('The cartesian action timed-out')
        return None


def QuaternionNorm(Q_raw):
    qx_temp, qy_temp, qz_temp, qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp * qx_temp + qy_temp * qy_temp + qz_temp * qz_temp + qw_temp * qw_temp)
    qx_ = qx_temp / qnorm
    qy_ = qy_temp / qnorm
    qz_ = qz_temp / qnorm
    qw_ = qw_temp / qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_


def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_, ty_, tz_]
    return EulerXYZ_


def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_


def update_current_pose():
    topic_address = '/' + prefix + 'driver/out/tool_pose'
    rospy.Subscriber(name=topic_address, data_class=geometry_msgs.msg.PoseStamped, callback=pose_callback)


def pose_callback(msg):
    global current_pose
    position_ = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    orientation_ = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    current_pose = position_ + orientation_


if __name__ == '__main__':
    rospy.init_node('visual_servoing')
    update_current_pose()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    start_time = rospy.Time.now()
    duration = rospy.Duration(90)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = current_time - start_time

        if elapsed_time > duration:
            rospy.loginfo("Time limit reached.")
            break

        update_current_pose()
        try:
            trans = tfBuffer.lookup_transform("world", "desired_end_effector", rospy.Time(0))
            position = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            orientation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            pose = position + orientation
            poses = [float(n) for n in pose]

            error = [x - y for x, y in zip(current_pose, poses)]
            error_norm = numpy.linalg.norm(error)
            # print(error_norm)
            if error_norm > 0:
                result = cartesian_pose_client(poses[:3], poses[3:])
                print('execute!')

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cannot lookup transform")
            rate.sleep()
            continue