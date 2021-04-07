# PEPPER-SLAM_Navigation_Bot
A differential drive mobile robot that can perform SLAM (Simultaneous Localization and Mapping) and Autonomous Navigation using navigation stack and avoid obstacles in a room.

## System Requirements: ##
1) Ubuntu 18.04 (Bionic Beaver) OS
2) ROS Melodic Morenia Framework
3) Python 2.7
4) Gazebo Simulator with ROS integration

**Setting up the Packages:**<br/>
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
1) Connect the Arduino Mega in the Robot to the Laptop via USB Cable and run the `chmod 777 (port name)` command to grant data transfer permissions.
2) Upload the following Arduino Sketch into the Ardino Mega of the Robot: `/catkin_ws/src/ros_arduino/ros_arduino_base/firmware/two_wheel_base/two_wheel_base.ino`
3) Plug in the USB port of the on-board Microsoft Kinect on the Robot into the laptop and run the `chmod 777 (port name)` command again on the port of the Kinect.
4) Run the following commands each in individual terminal tabs/windows, to start the SLAM process:
   ```
   roslaunch ros_arduino_base base_bringup.launch
   roslaunch pepper urdf.launch
   roslaunch pepper pepper_rtab.launch
   roslaunch teleop_twist_joy teleop.launch
   ```
 5) Run `rviz` command, and select the rostopics you would like to visualize during SLAM.
 6) Now drive the Bot around using the joystick and map the entire room. The mapped areas can be visualized by adding the `/map` topic in the rviz window.
 7) Finally after mapping, run the following command to save the map offline, for future use such as navigation:<br/>  
    `rosrun map_server map_saver -f $(find pepper)/maps/map`
