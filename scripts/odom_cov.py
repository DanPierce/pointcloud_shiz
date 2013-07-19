#!/usr/bin/env python

import roslib; roslib.load_manifest('pointcloud_shiz')

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

def talker(message):
    corrected = Odometry()
    corrected.header.seq = message.header.seq
    corrected.header.stamp = rospy.Time.now()
    corrected.header.frame_id = message.header.frame_id #base_footprint #message.header.frame_id
    corrected.child_frame_id = message.child_frame_id #"base_footprint"
    corrected.pose.pose = message.pose.pose

    corrected.pose.covariance = [0.05, 0, 0, 0, 0, 0,
                                 0, 0.05, 0, 0, 0, 0,
                                 0, 0, 0.05, 0, 0, 0,
                                 0, 0, 0, 0.001, 0, 0,
                                 0, 0, 0, 0, 0.001, 0,
                                 0, 0, 0, 0, 0, .5]

    corrected.twist.twist = message.twist.twist

    corrected.twist.covariance = [0.05, 0, 0, 0, 0, 0,
                                 0, 0.05, 0, 0, 0, 0,
                                 0, 0, 0.05, 0, 0, 0,
                                 0, 0, 0, 0.001, 0, 0,
                                 0, 0, 0, 0, 0.001, 0,
                                 0, 0, 0, 0, 0, .5]

    pub = rospy.Publisher('odom_w_cov', Odometry)
    pub.publish(corrected)

def listener():
    rospy.init_node('odom_cov', anonymous=True)
    rospy.Subscriber("atrv_node/odom", Odometry, talker)
    rospy.spin()

if __name__ == '__main__':
    
    listener()


