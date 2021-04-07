# PEPPER-SLAM_Navigation_Bot
A differential drive mobile robot that can perform SLAM (Simultaneous Localization and Mapping) and Autonomous Navigation using navigation stack and avoid obstacles in a room.

## System Requirements: ##
1) Ubuntu 18.04 (Bionic Beaver) OS
2) ROS Melodic Morenia Framework
3) Python 2.7
4) Gazebo Simulator with ROS integration

#### Setting up the Packages: ####
Create a catkin workpace and clone all the contents in `/catkin_ws/src` directory of this repository into its source folder. And then run the `catkin_make` command.

## Part-1: Running the Simulations ##
For running the simulations follow the instructions provided in the official Turtlebot3 documantaion, as follows:
- Visit the Turtlebot3 Simulation e-manual by clicking on the link: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
- Run the commands present in the following sections:
  - 6.1) Gazebo Simulation
  - 6.2) SLAM Simulation
  - 6.3) Navigation Simulation
- Make sure to check the "Melodic" tab on top of each page to run the compatible ROS commands.

## Part-2: Running SLAM on Robot ##
1) Connect the Arduino Mega in the Robot to the Laptop via USB Cable and run the `chmod 777 (port name)` 
1) Upload the following Arduino Sketch into the 
