#!/usr/bin/env python

import roslib; roslib.load_manifest('pointcloud_shiz')

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

def talker(message):
    corrected = Imu()
    corrected.header.seq = message.header.seq
    corrected.header.stamp = rospy.Time.now()
    corrected.header.frame_id = message.header.frame_id

    # Transpose
    corrected.linear_acceleration.x = message.linear_acceleration.y
    corrected.linear_acceleration.y = -message.linear_acceleration.x
    corrected.linear_acceleration.z = -message.linear_acceleration.z

    corrected.angular_velocity.x = message.angular_velocity.x
    corrected.angular_velocity.y = message.angular_velocity.y
    corrected.angular_velocity.z = -message.angular_velocity.z


    corrected.orientation.x = 1
    corrected.orientation.y = 0
    corrected.orientation.z = 0
    corrected.orientation.w = 0
    corrected.orientation_covariance = [99999, 0, 0,
                                         99999, .1, 0,
                                         0, 0, 99999]
    
    corrected.angular_velocity_covariance = [.000004, 0, 0,
                                              0, .000004, 0,
                                              0, 0, .000004]

    corrected.linear_acceleration_covariance = [.000025, 0, 0,
                                                 0, .000025, 0,
                                                 0, 0, .000025]

    pub = rospy.Publisher('imu_w_cov', Imu)
    
    
    #print >> f, "%f %f %f %f %f %f \n" % (message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z, message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z)


    pub.publish(corrected)

def listener():
    rospy.init_node('imu_cov', anonymous=True)
    rospy.Subscriber("/xbow440_node/imu/data", Imu, talker)
    #if rospy.is_shutdown():
        #f.close()
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    #f = open("/home/gavlab/devel/vizdemo_ws/pointcloud_shiz/data/imu_data.txt", "w")
    listener()


