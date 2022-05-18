#!/usr/bin/env python

import rospy
import sys
import os
import math
import csv

from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates 
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped

car_name        = str(sys.argv[1])
trajectory_name = str(sys.argv[2])

plan = []

index_pub = rospy.Publisher('/{}/purepursuit_control/index_nearest_point'.format(car_name), Int64, queue_size = 1)
min_pose_pub  = rospy.Publisher('/{}/purepursuit_control/visualize_nearest_point'.format(car_name), PoseStamped, queue_size = 1)

def construct_path():
    file_path = os.path.expanduser('/path_following_ws/src/path_following/path/{}.csv'.format(trajectory_name))

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ';')
        for waypoint in csv_reader:
            plan.append(waypoint)

    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

def odom_callback(data):
    min_index      = Int64()
    curr_x         = data.pose[1].position.x
    curr_y         = data.pose[1].position.y
    min_index.data = find_nearest_point(curr_x, curr_y)
    index_pub.publish(min_index)
    print"position recieved "
    print curr_x
    print curr_y   	

    pose                 = PoseStamped()
    pose.pose.position.x = plan[min_index.data][1]
    pose.pose.position.y = plan[min_index.data][2]
    min_pose_pub.publish(pose)

def find_nearest_point(curr_x, curr_y):
    ranges = []
    for index in range(0, len(plan)):
        eucl_x = math.pow(curr_x - plan[index][1], 2)
        eucl_y = math.pow(curr_y - plan[index][2], 2)
        eucl_d = math.sqrt(eucl_x + eucl_y)
        ranges.append(eucl_d)
    return(ranges.index(min(ranges)))

if __name__ == '__main__':
    try:
        rospy.init_node('nearest_pose_isolator', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()
        rospy.Subscriber('/gazebo/model_states', ModelStates, odom_callback)
	print"node running test"
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
