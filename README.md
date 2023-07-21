# Line Following Car using ROS 2

This repository contains the code and instructions to build a line-following car using ROS 2 (Robot Operating System 2). The line-following car is capable of autonomously following a line path. The simulation is done using Gazebo.

## To build
To build this project, follow the steps below:

1. Install ROS2 using this [link](https://docs.ros.org/en/humble/Installation.html). Create a ROS2 workspace:
```bash
mkdir ros2_ws/src
cd ros2_ws/src
```
2. Clone this repository to your ros2 workspace:
```bash
git clone https://github.com/HariprasathSivakumaar/ROS2-Line-Follower.git
```
3. Build ros2 packages for the project:
```bash
cd ros2_ws
colcon build
```
4. Source the setup files:
```bash
source install/setup.bash
```
5. Launch the Simulation:
```bash
ros2 launch prius_line_following car_on_track.launch.py 
```
This command will launch the robot model in Gazebo. 

6. Run the line following script:
```bash
ros2 run prius_line_following line_follower
```
