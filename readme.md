# Autonomous Surface Vehicle (ASV)
Building an Autonomous Surface Vehicle (ASV) for smart sustainalble shellfish aquaculture management

## Installing the dependencies
*To install MAVROS*
```
https://masoudir.github.io/mavros_tutorial/
```

## Starting the ArduRover firmware
*Connect the pixhawk via USB cable or telemetry*
```
roslaunch asv_build terpbot.launch 

```
*To execute GPS aided autonomous navigation*
```
rosrun asv_build gps_nav_2.py 

```
*To execute a square pattern*
```
rosrun asv_build square.py 

```

