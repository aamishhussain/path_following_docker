#! /usr/bin/env python

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
import sensor_msgs.msg as sensor_msgs

"""
The bno055_usb_stick node sets first covariance entries to -1. This wrongly
indicates that the corresponding measurement is not available.

See
http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
"""

if __name__ == '__main__':
    rospy.init_node('imu_fix')

    pub = rospy.Publisher('imu_out', numpy_msg(sensor_msgs.Imu), queue_size=1)

    def callback(msg):
        msg.orientation_covariance = np.array(msg.orientation_covariance)
        msg.orientation_covariance[0] = 0
        msg.angular_velocity_covariance = np.array(
            msg.angular_velocity_covariance)
        msg.angular_velocity_covariance[0] = 0
        msg.linear_acceleration_covariance = np.array(
            msg.linear_acceleration_covariance)
        msg.linear_acceleration_covariance[0] = 0
        pub.publish(msg)

    sub = rospy.Subscriber('imu_in', numpy_msg(sensor_msgs.Imu), callback,
                           queue_size=1)

    rospy.spin()