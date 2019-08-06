#!/usr/bin/env python
import time
# import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Pose

from geometry_msgs.msg import Point

# For teleoperation
import sys, select, termios, tty 
import numpy as np

from sensor_msgs.msg import JointState # To receive the current state


def callback(data):

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.position.x)

def listener():
      # In ROS, nodes are uniquely named. If two nodes with the same
     # name are launched, the previous one is kicked off. The
     # anonymous=True flag means that rospy will choose a unique
     # name for our 'listener' node so that multiple listeners can
     # run simultaneously.
     rospy.init_node('listener', anonymous=True)

     rospy.Subscriber("/pose_topic",PoseStamped,callback, queue_size=10)

   # spin() simply keeps python from exiting until this node is stopped
     rospy.spin()

if __name__ == '__main__': 
    settings = termios.tcgetattr(sys.stdin)
    listener()
