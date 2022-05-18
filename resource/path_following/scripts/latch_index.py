#!/usr/bin/env python

import rospy
import sys
import os
import math
import csv


from std_msgs.msg import Int64
from std_msgs.msg import Bool

car_name        = str(sys.argv[1])
trajectory_name = str(sys.argv[2])

plan = []

global current_index

current_index = 0

index_pub = rospy.Publisher('/{}/purepursuit_control/latched_index'.format(car_name), Int64, queue_size = 1)
#min_pose_pub  = rospy.Publisher('/{}/purepursuit_control/visualize_nearest_point'.format(car_name), PoseStamped, queue_size = 1)

def construct_path():
    file_path = os.path.expanduser('/home/sim/f1-10-simulator/catkin_ws/src/path_following/path/{}.csv'.format(trajectory_name))

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ';')
        for waypoint in csv_reader:
            plan.append(waypoint)

    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])  	

def latch_callback(data):
    global current_index
    if current_index == (len(plan)-1) and data.data:
        current_index = 0
    elif current_index != (len(plan)-1) and data.data:
        current_index= current_index + 1        
    index_pub.publish(current_index)    
    
if __name__ == '__main__':
    try:
        #rospy.init_node('nearest_pose_isolator', anonymous = True)
        rospy.init_node('latch', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()
        rospy.Subscriber('/change_the_index',Bool,latch_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
