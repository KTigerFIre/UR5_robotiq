#!/usr/bin/python
#
# Send a value to change the opening of the Robotiq gripper using an action
#

import argparse
import string
import rospy
import actionlib
from gazebo_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

def callback(data):
    rate = rospy.Rate(100) # 50hz
    #print(data.name[2])
    print(data.pose[2])
    #print(data.name[3])
    #print(data.pose[3])
    pub = rospy.Publisher("table_topic",PoseStamped,queue_size=1)
    table = PoseStamped()
    table.header =Header()
    table.header.frame_id = 'world'
    table.header.stamp = rospy.Time.now()
    table.pose = Pose()
    table.pose = data.pose[1]
    pub.publish(table)
    pub2 = rospy.Publisher("object_topic",PoseStamped,queue_size=1)
    object = PoseStamped()
    object.header = Header()
    object.header.frame_id = 'world'
    object.header.stamp = rospy.Time.now()
    object.pose = Pose()
    object.pose = data.pose[2]
    pub2.publish(object)

    rate.sleep()

def reader():
    rospy.init_node("test_topic", anonymous=True, disable_signals=True)  
    rospy.Subscriber("gazebo/model_states", ModelStates, callback, queue_size=1)
    rospy.spin()

def main():

    try:
              
        
        print "Waiting for topic..."

        reader()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
if __name__ == '__main__': 
    main()
