#!/usr/bin/env python

from pickle import TRUE
from tkinter.font import BOLD
import rospy
import sys
import os
import math
import csv

from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates 
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

car_name        = str(sys.argv[1])
trajectory_name = str(sys.argv[2])

plan = []

#index_pub = rospy.Publisher('/{}/purepursuit_control/index_nearest_point'.format(car_name), Int64, queue_size = 1)
#min_pose_pub  = rospy.Publisher('/{}/purepursuit_control/visualize_nearest_point'.format(car_name), PoseStamped, queue_size = 1)
index_change_pub = rospy.Publisher('/{}/purepursuit_control/change_the_index'.format(car_name), Bool, queue_size = 1)



global current_index
global current_pose
global threshold 

threshold = 0.2


def construct_path():
    file_path = os.path.expanduser('/home/sim/f1-10-simulator/catkin_ws/src/path_following/path/{}.csv'.format(trajectory_name))

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ';')
        for waypoint in csv_reader:
            plan.append(waypoint)

    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

def index_callback(data):
    global current_index
    current_index=data.data

def check_threshold(curr_x, curr_y):
    global current_index
    global threshold 
    change_index=0
    
    eucl_x = math.pow(curr_x - plan[current_index][1], 2)
    eucl_y = math.pow(curr_y - plan[current_index][2], 2)
    eucl_d = math.sqrt(eucl_x + eucl_y)
    if threshold <= eucl_d:
        change_index=1
    else: 
        change_index=0  
    return(change_index)            

def odom_callback(data):
    change_index      = Bool()
    curr_x         = data.pose[1].position.x
    curr_y         = data.pose[1].position.y
    change_index.data = check_threshold(curr_x, curr_y)
    index_change_pub.publish(change_index) 

if __name__ == '__main__':
    try:
        rospy.init_node('nearest_pose_isolator', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()
        rospy.Subscriber('/{}/purepursuit_control/latched_index'.format(car_name),Int64,index_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, odom_callback)
	    #print "node running test"
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
