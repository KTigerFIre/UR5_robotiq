# UR5_robotiq
Gazebo simulation for UR5 with robotiq 3finger gripper

This is ros pakages which I used to make Gazebo simulation for my project.
Three different simulations are prepared.
 - Single UR5 (no gripper) : summer_robot.launch
 - Twin UR5 (no gripper) : summer_twin.launch
 - Single UR5 (with gripper) : ur5_robotiq.launch
 
The robotiq 3-f gripper is adopted ros_control based controller which is not same as original robotiq plugin. 
The reason why I apply the ros_contorol is becuase the original plugin from robotiq package is not good enough for simulation.

https://www.youtube.com/watch?v=4zsZUm7T3LA
On youtube, the example of simulation is uploaded.

##Installastion

  1. Copy the project on your catkin workspace ( you need to install ROS prioly)
   -cd catkin_ws
   -cd src
   -git clone

   2. Install depenancies ( !important)
   -rosdep update
   -cd ..
   -rosdep update rosdep install --rosdistro kinetic --ignore-src --from-paths src
   -catkin_make

   3.Launch simulation
   -roslaunch summer_gazebo ur5_robotiq.launch 
   -rosrun summer_gazebo gazebo_teleop_key_xyz.py
   -rosrun summer_gazebo gripper_test.py
