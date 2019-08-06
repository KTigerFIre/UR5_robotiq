#!/usr/bin/python
#
# Send a value to change the opening of the Robotiq gripper using an action
#

import string
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from gazebo_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ur_kinematics import Kinematics
import math


def obstacle():
    data = rospy.Subscriber("gazebo/model_states", ModelStates, queue_size=1)
    obstacle = data.pose[2]


def callback(data):
    ur_status = data
    robot = PoseStamped()
    robot.header =Header()
    robot.header.frame_id = 'base_link'
    robot.header.stamp = rospy.Time.now()
    joint_pose = ur_status.actual.positions
    rate = rospy.Rate(100) # 50hz
    kin = Kinematics('ur5')
    pose_xyz = kin.forward(joint_pose) 
    current_xyz = pose_xyz[0:3,3:4]
    
    pub = rospy.Publisher("ur5_ee_topic",PoseStamped,queue_size=1)
    robot.pose = Pose()
    robot.pose.position = Point()
    robot.pose.position.x = current_xyz[0]
    robot.pose.position.y = current_xyz[1]
    robot.pose.position.z = current_xyz[2]
    pub.publish(robot)
    rospy.loginfo(robot)
    rate.sleep()
def reader():
    rospy.init_node("test_topic", anonymous=True, disable_signals=True)  
    rospy.Subscriber("arm_controller/state", JointTrajectoryControllerState, callback, queue_size=1)
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
