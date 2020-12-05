## Build Instructions
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
copy group4_final_project package here
cd ..
catkin_make
```

## Running the code


Instructions for running package:
```
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch group4_final_project group4_final_project.launch


#Start moveit for arm1 (https://bitbucket.org/osrf/ariac/wiki/2019/tutorials/moveit_interface)
source ~/ariac_ws/install/setup.bash
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1

#Start moveit for arm2 (https://bitbucket.org/osrf/ariac/wiki/2019/tutorials/moveit_interface)
source ~/ariac_ws/install/setup.bash
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm2

#Start node
rosrun group4_final_project main_node
