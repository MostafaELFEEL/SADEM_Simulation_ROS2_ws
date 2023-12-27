# SADEM_Simulation_ROS2

Check out our primary project repository: [SADEM_ws](https://github.com/RoniEmad/SADEM_ws)

## Overview
SADEM (Swarm of Autonomous Drones for Environment Mapping) was a project that aimed to develop a swarm of autonomous drones that can generate detailed 3D visualizations of any indoor or outdoor area. The system consists of several drones that are equipped with stereo cameras and an on-ground base station that performs the heavy computation needed to generate the visualizations. The drones navigate and localize themselves using a combination of inertial navigation and visual odometry, while communication and coordination between the drones are enabled through the use of wireless protocols and algorithms.

This simulation was made using CoppeliaSim to test cascaded PID control.




## Setup and Execution

### Environment Setup:

1. **Create Workspace Directory:**
```bash
mkdir sadem_simulation_ros2_ws && cd sadem_simulation_ros2_ws
```

2. **Clone Repository:**
```bash
git clone https://github.com/MostafaELFEEL/SADEM_Simulation_ROS2.git
```

3. **Build Workspace:**
```bash
colcon build
```

### Coppeliasim Configuration:

- Download [Coppeliasim edu v.4.5.1](https://www.coppeliarobotics.com/files/V4_5_1_rev4/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu22_04.tar.xz).
- Go to coppeliasim folder and search for interfaces.txt
- Replace interfaces.txt with the one in this repository.
- Add these lines to .bashrc and make sure to change **<Path_to_Coppeliasim_Folder>** with Coppeliasim folder path.
```bash
export VREP_ROOT=~/<Path_to_Coppeliasim_Folder>
export COPPELIASIM_ROOT_DIR=~/<Path_to_Coppeliasim_Folder>
```
- Open Terminal:
```bash
cd ~/<Path_to_Coppeliasim_Folder>
colcon build --packages-select sim_ros2_interface
```

  

### Launch Simulation:

- **Default Simulation:**
```bash
ros2 launch sadem_ros2_simulation sadem_sim.py
ros2 run sadem_ros2_simulation controller
```

- **Specific Map Simulation:**
```bash
ros2 launch sadem_ros2_simulation sadem_sim.py map_number:=1
ros2 run sadem_ros2_simulation controller
```

### Individual Component Execution:

```bash
ros2 run sadem_ros2_simulation a_star --ros-args -p map_number:=1
ros2 run sadem_ros2_simulation path_to_goal
ros2 run sadem_ros2_simulation motors
ros2 run sadem_ros2_simulation controller
```

## Additional Resources:

- **Drone Control Loop**:
  
![image](https://github.com/MostafaELFEEL/SADEM_Simulation_ROS2/assets/106331831/19c914fe-567b-44d9-8321-913ad5a1c02a)

- **System Block Diagram**:
  
![Diagram](https://github.com/MostafaELFEEL/SADEM_Simulation_ROS2/assets/106331831/5fa63698-972d-48c3-983e-e6a25ebbe7b7)
  
- **RVIZ2 Screenshot**:

![RVIZ2](https://github.com/MostafaELFEEL/SADEM_Simulation_ROS2/assets/106331831/b9ef276b-6557-4b79-bbc1-a810e92cbcea)

- **Simulation Video**:
  
https://github.com/MostafaELFEEL/SADEM_Simulation_ROS2/assets/106331831/0030b97c-3c4c-49c0-8d76-be0e3f578a83
  
- **Path Planning Video**:
  
https://github.com/MostafaELFEEL/SADEM_Simulation_ROS2/assets/106331831/c3ebf018-4172-4cf0-a97b-9f1bca4036a6
