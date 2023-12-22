# SADEM_Simulation_ROS2_ws
## About:

launch simulaion:
bash

'''
$ros2 launch sadem_ros2_simulation sadem_sim.py 
$ros2 run sadem_ros2_simulation controller
'''

Launch simulation on different map:
$ros2 launch sadem_ros2_simulation sadem_sim.py map_number:=1
$ros2 run sadem_ros2_simulation controller


there are 7 maps and 1 is the default map if there was no arguments

to run the codes individually:
$ros2 run sadem_ros2_simulation a_star

or you can run a_star with different map using arguments:
ros2 run sadem_ros2_simulation a_star --ros-args -p map_number:=1

then run:
$ros2 run sadem_ros2_simulation path_to_goal

$ros2 run sadem_ros2_simulation motors

$ros2 run sadem_ros2_simulation controller


i will edit the readme 
