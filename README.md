# SADEM_Simulation_ROS2
## About:
SADEM or Swarm of Autonomous Drones for Envirment Mapping was a project that aimmed to develop a swarm of autonomous drones that can generate detailed 3D visualizations of any indoor or outdoor area. The system consists of several drones that are equipped with stereo cameras and an on-ground base station that performs the heavy computation needed to generate the visualizations. The drones navigate and localize themselves using a combination of inertial navigation and visual odometry, while communication and coordination between the drones are enabled through the use of wireless protocols and algorithms.

Check out our main project repo https://github.com/RoniEmad/SADEM_ws

this simulation was used to to test the controller which is cascaded PID control.

how to create the enviroment:
mkdir sadem_simulation_ros2_ws && cd sadem_simulation_ros2_ws
git clone 







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
