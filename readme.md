# Autonomous Surface Vehicle (ASV)
Building an Autonomous Surface Vehicle (ASV) for smart sustainalble shellfish aquaculture management
![](ASV.jpeg)
## Installing the dependencies
*MAVROS:*
*to install MAVROS*
https://masoudir.github.io/mavros_tutorial/ <br>
*ROS Noetic*<br>
*Python3 (recommended)*
## Starting the ArduRover firmware
*Connect the pixhawk via USB cable or telemetry*
```
roslaunch asv terpbot.launch 
```
## ROS Control
*To execute GPS aided autonomous navigation. Edit the code to enter the required waypoints for navigation and then*
```
rosrun asv gps_nav_2.py 
```
*To execute a square pattern in guided mode*
```
rosrun asv guided.py 
```
*To execute a lawnmover pattern in guided mode*
```
rosrun asv guided_lawn.py 
```

