#!/usr/bin/env python

import roslib; roslib.load_manifest('pointcloud_shiz')

import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import String

def callback(msg):
    pub = rospy.Publisher("vo_covar", Odometry)

    corrected = Odometry()
    corrected.header.seq = msg.header.seq
    corrected.header.stamp = rospy.Time.now()
    corrected.header.frame_id = msg.header.frame_id
    corrected.child_frame_id = "base_footprint"
    corrected.pose.pose = msg.pose.pose
    corrected.pose.covariance = [.01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01,
                                 .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01,
                                 .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01] 

    corrected.twist.twist = msg.twist.twist
    corrected.twist.covariance = [.01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01,
                                 .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01,
                                 .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01, .01] 

    pub.publish(corrected)

def listener():
        rospy.Subscriber("/viso/odometry", Odometry, callback)
        rospy.spin()

    

if __name__ == '__main__':
    rospy.init_node('vo_covariance')
    try:
        listener()
    except rospy.ROSInterruptException: pass
