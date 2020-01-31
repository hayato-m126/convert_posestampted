#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class ConvertPoseStampted:
    def __init__(self):
        self.__poseStampted = PoseStamped()
        self.__sub_pose_stamped_covariance = rospy.Subscriber("gnss_pose_covariance", PoseWithCovarianceStamped, self.__pose_callback)
        self.__pub_pose_stamped = rospy.Publisher("gnss_pose", PoseStamped, queue_size=1)

    def __pose_callback(self, poseWithCovearianceStampted):
        self.__poseStampted.header = poseWithCovearianceStampted.header
        self.__poseStampted.pose = poseWithCovearianceStampted.pose.pose
        self.__poseStampted.pose.position.x -= 19034.12
        self.__poseStampted.pose.position.y -= 127120.2
        self.__pub_pose_stamped.publish(self.__poseStampted)

def main():
    rospy.init_node('convert_posestamped')
    converter = ConvertPoseStampted()
    rospy.spin()

if __name__ == '__main__':
    main()