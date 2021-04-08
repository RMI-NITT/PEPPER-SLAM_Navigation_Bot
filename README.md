# PEPPER-SLAM_Navigation_Bot
A Differential Drive Mobile Robot that can perform SLAM (Simultaneous Localization and Mapping) and Autonomous Navigation using navigation stack and avoid obstacles in a room.

**NOTE: Scroll Down to view the Photo Gallery of the Robot and its functionalities.**

## Demonstration Videos (YouTube):
1) Navigation Simulation on Gazebo: https://youtu.be/JQFO9v_3PpE
2) Performing SLAM with PEPPER: https://youtu.be/AsfRXxU7h94
3) Autonomous Navigation Demonstration: https://youtu.be/RHS6B5DbNY4

Full Playlist (including old videos): https://youtube.com/playlist?list=PL44ElmNkyTvBqNAVxaPmKnhxJGcgDSsyj

## System Requirements: ##
1) Ubuntu 18.04 (Bionic Beaver) OS
2) ROS Melodic Morenia Framework
3) Python 2.7
4) Gazebo Simulator with ROS integration

**Setting up the Packages:**<br/>
Create a catkin workspace and clone all the contents in `/catkin_ws/src` directory of this repository into its source folder. And then run the `catkin_make` command.

## Part-1: Running the Simulations ##
For running the simulations follow the instructions provided in the official Turtlebot3 documentation, as follows:
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
 5) Run `rviz` command, and select the ros topics you would like to visualize during SLAM.
 6) Now drive the Bot around using the joystick and map the entire room. The mapped areas can be visualized by adding the `/map` topic in the rviz window.
 7) Finally after mapping, run the following command to save the map offline, for future use such as navigation:  
    `rosrun map_server map_saver -f $(find pepper)/maps/map`

## Part-3: Implementing Autonomous Navigation ##
1) Close all the running ros nodes and run the following launch files, to enable point to point navigation:
    ```
    roslaunch ros_arduino_base base_bringup.launch
    roslaunch pepper pepper_loc_rtab.launch
    roslaunch pepper pepper_navigation.launch
    roslaunch teleop_twist_joy teleop.launch
    ```
2) Using the joystick, drive the Bot shortly facing a wall/obstacle/corner until the rtab-map package auto-localizes the Bot's location on the map, using the scan data obtained from the Kinect.
3) Now select the "Set Goal" option on the toolbar at the top of the rviz window, and click a point on the map where you want the Bot to autonomously navigate to.
4) If you have followed the instructions correctly, you can now observe the Bot autonomously doing the path planning and navigating to the Goal point that you chose.
5) After the Bot reaches the destination point, you can select your next desired goal and repeat Steps 3, 4 & 5 as many times as you want.

# Photo Gallery

## The Robot: ##
![PEPPER-Front](/pictures/PEPPER-Front.jpg?raw=true "PEPPER Front")
![PEPPER-Back](/pictures/PEPPER-Back.jpg?raw=true "PEPPER Back")
![PEPPER-Circuit](/pictures/PEPPER-Circuit.jpg?raw=true "PEPPER Circuit")
![PEPPER-Bottom](/pictures/PEPPER-Bottom.jpg?raw=true "PEPPER Bottom")

## 1) Simulation on Gazebo: ##
![Simulation-1](/pictures/Simulation-1.jpg?raw=true "View from North Room")
![Simulation-2](/pictures/Simulation-2.jpg?raw=true "View from North Hall")
![Simulation-3](/pictures/Simulation-3.jpg?raw=true "View from South Hall")

## 2) Performing SLAM with PEPPER: ##
![SLAM-1](/pictures/SLAM-1.jpg?raw=true "SLAM 1")
![SLAM-2](/pictures/SLAM-2.jpg?raw=true "SLAM 2")
![SLAM-3](/pictures/SLAM-3.jpg?raw=true "SLAM 3")

## 3) Autonomous Navigation: ##
![Navigation-1](/pictures/Navigation-1.jpg?raw=true "Area to Navigate")
![Navigation-2](/pictures/Navigation-2.jpg?raw=true "Assigning Goal in RViz")
![Navigation-3](/pictures/Navigation-3.jpg?raw=true "Path Planned to Goal")
![Navigation-4](/pictures/Navigation-4.jpg?raw=true "Robot reached the Goal")
