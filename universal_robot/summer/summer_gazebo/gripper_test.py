#!/usr/bin/env python2
# Author: Aleksandrs Bogucarskis; TODO: Refactor this code!
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Header
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

def main():
    rospy.init_node('down_node')
    pub1 = rospy.Publisher('robotiq_controller/command',
                              JointTrajectory,
                              queue_size=1)
    rate = rospy.Rate(100) # 50hz
    traj = JointTrajectory()
    traj.header = Header()
    traj.joint_names = ['finger_1_joint_1', 'finger_1_joint_2', 'finger_1_joint_3',
                        'finger_2_joint_1', 'finger_2_joint_2', 'finger_2_joint_3',
                        'finger_middle_joint_1','finger_middle_joint_2','finger_middle_joint_3',
                        'palm_finger_1_joint','palm_finger_2_joint']
    traj.header.stamp = rospy.Time.now()
    pts = JointTrajectoryPoint()  ## same as 'command = outputMsg.SModel_robot_output()'
    # Publisher to Unity
    pub2= rospy.Publisher('float_unity0', Float64,queue_size=1)
    pub3= rospy.Publisher('float_unity1', Float64,queue_size=1)
    pub4= rospy.Publisher('float_unity2', Float64,queue_size=1)

  
    print("Waiting...")
  

    print("Activated gripper!")

    #scaleFactor = 1023/255
    scaleFactor = (1.1-0.05)/500.0  ##(max 1.1,min 0.05)
    input = 20
    while not rospy.is_shutdown():
       try:
         rec_int = [input, input, input] #Range: 0 - 500
         print(rec_int)
         #print(float(rec_int[1]))
         #print(scaleFactor)

         finger_1 = (0.05+rec_int[0] * scaleFactor)
         finger_2 = (0.05+rec_int[1] * scaleFactor)
         finger_middle = (0.05+rec_int[2] * scaleFactor)
         pose = [finger_1, finger_1, (-1.22+finger_1), finger_2, finger_2, (-1.22+finger_2), finger_middle, finger_middle, (-1.22+finger_middle), 0.19, -0.19]
         pts.positions = pose
         pts.time_from_start = rospy.Duration(0.1)
         traj.points = []
         traj.points.append(pts)
         rate.sleep()
         pub1.publish(traj)#Range of gripper : 0.05 between 1.1 (rad)
         print("Finger A: " + str(finger_1))
         print("Finger B: " + str(finger_2))
         print("Finger C: " + str(finger_middle))
         pub2.publish(rec_int[0])
         pub3.publish(rec_int[1])
         pub4.publish(rec_int[2])
         print("Writing...")
         
       except ValueError: #TODO: implement better error detection
         print("ValueError")

if __name__ == "__main__":
    main()

