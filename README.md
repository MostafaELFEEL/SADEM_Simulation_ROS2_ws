# SADEM_Simulation_ROS2

Check out our main project repo https://github.com/RoniEmad/SADEM_ws

## About:
SADEM or Swarm of Autonomous Drones for Envirment Mapping was a project that aimed to develop a swarm of autonomous drones that can generate detailed 3D visualizations of any indoor or outdoor area. The system consists of several drones that are equipped with stereo cameras and an on-ground base station that performs the heavy computation needed to generate the visualizations. The drones navigate and localize themselves using a combination of inertial navigation and visual odometry, while communication and coordination between the drones are enabled through the use of wireless protocols and algorithms.
this simulation was used to test the controller which is cascaded PID control.

Creating the enviroment:
```bash
mkdir sadem_simulation_ros2_ws && cd sadem_simulation_ros2_ws
git clone https://github.com/MostafaELFEEL/SADEM_Simulation_ROS2.git
cd ~/sadem_simulation_ros2_ws
colcon build
```


setting up coppeliasim:
Install Coppeliasim edu v.4.5.1
```bash
https://www.coppeliarobotics.com/files/V4_5_1_rev4/CoppeliaSim_Player_V4_5_1_rev4_Ubuntu22_04.tar.xz
```
check for the supported msgs:
search for interfaces.txt file in coppeliasim folder and add missing message types then colcon build.

to start the simulation:
launch simulaion:
```bash
ros2 launch sadem_ros2_simulation sadem_sim.py
```
```
ros2 run sadem_ros2_simulation controller
```

Launch simulation on different map:
```
ros2 launch sadem_ros2_simulation sadem_sim.py map_number:=1
```
```
ros2 run sadem_ros2_simulation controller
```

there are 7 maps and 1 is the default map if there was no arguments.

to run the codes individually:
```
ros2 run sadem_ros2_simulation a_star
```
or you can run a_star with different map using arguments:
```
ros2 run sadem_ros2_simulation a_star --ros-args -p map_number:=1
```
then run:
```
ros2 run sadem_ros2_simulation path_to_goal
```
```
ros2 run sadem_ros2_simulation motors
```
```
ros2 run sadem_ros2_simulation controller
```

simulation video:

https://github.com/MostafaELFEEL/SADEM_Simulation_ROS2/assets/106331831/0030b97c-3c4c-49c0-8d76-be0e3f578a83

path planning video:

https://github.com/MostafaELFEEL/SADEM_Simulation_ROS2/assets/106331831/c3ebf018-4172-4cf0-a97b-9f1bca4036a6

rviz2:
![Screenshot from 2023-12-20 19-30-03](https://github.com/MostafaELFEEL/SADEM_Simulation_ROS2/assets/106331831/b9ef276b-6557-4b79-bbc1-a810e92cbcea)

system block diagram:

![image](https://github.com/MostafaELFEEL/SADEM_Simulation_ROS2/assets/106331831/5fa63698-972d-48c3-983e-e6a25ebbe7b7)
