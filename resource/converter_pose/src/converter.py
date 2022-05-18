#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import String

Skip=True
my_data=None
def callback(data):
    global my_data
    my_data=Pose()
    my_data.position.x=data.position.x
    my_data.position.y=data.position.y
    my_data.position.z=data.position.z
    my_data.orientation.x=data.orientation.x
    my_data.orientation.y=data.orientation.y
    my_data.orientation.z=data.orientation.z
    my_data.orientation.w=data.orientation.w
    goal_publisher = rospy.Publisher("robot_pose_2", PoseStamped, queue_size=5)
    goal = PoseStamped()
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    
    goal.pose.position = my_data.position
    #goal.pose.position.y = 2.0
    #goal.pose.position.z = 0.0
    
    goal.pose.orientation = my_data.orientation
    #goal.pose.orientation.y = 0.0
    #goal.pose.orientation.z = 0.0
    #goal.pose.orientation.w = 1.0
    
    goal_publisher.publish(goal)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s",my_data)

def publish():
    global Skip
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("robot_pose",Pose, callback)
    rospy.spin()
    
if __name__ == '__main__':
    publish()
