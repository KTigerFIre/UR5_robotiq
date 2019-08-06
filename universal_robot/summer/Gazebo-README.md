#UR5 twins for summer internship
---
## Overview
cd $HOME/catkin_ws/src
Move universal_robot folder into your src folder
cd $HOME/catkin_ws
# checking dependencies (again: replace '$ROS_DISTRO' with the ROS version you are using)
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src

#building
catkin_make

#Launch simulation
roslaunch summer_gazebo summer_robot.launch
roslaunch summer_gazebo ur5_robotiq.launch
#Teleop keyboard#
rosrun summer_gazebo gazebo_teleop_key_xyz.py
rosrun summer_gazebo gazebo_teleop_deep.py
rosrun summer_gazebo ee_topic.py
rosrun summer_gazebo table_topic.py


        



