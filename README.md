# Roberto the robot!

Roberto the robot is a 4 wheeled robot, built for DTU's annual Robocup competition. The competition requires mobile robots to complete a track with a number of different obstacles and tasks to be completed. More info can be found on http://www.robocup.dtu.dk/ (Only available in Danish).

Roberto controls it's wheels in pairs (Left and Right respectively) in order to allow pivot sterring, turning around center and sideways locomotion.

![DriveModes](http://i.imgur.com/0CxI1aK.png)

## Folder structure of Roberto

The folder structure of Roberto is following the generic ROS folder structure. An example of this can be seen on https://github.com/davetcoleman/baxter_cpp/tree/hydro-devel
```
roberto_description		- Contains URDF description and 3d models of Roberto
roberto_gazebo			- Contains all the models, launch files and world files for simulating in gazebo
roberto_control			- Contains controller definitions and controller specific software
uC_software				- Contains all the software for the microcontrollers on Roberto
robocup 				- Contains information regarding DTU Robocup
```

## Prerequisites

### ROS

To set up a working ros install, with all required packages for Roberto to compile and run, start by installing ROS by following the tutorial on their site: (Roberto is only tested on ROS Indigo)
http://wiki.ros.org/indigo/Installation/Ubuntu

Furthermore a bunch of ros packages are needed in order for Roberto to run:
```ros-indigo-rosserial-client ros-indigo-rosserial-arduino ros-indigo-rosserial-server ros-indigo-rosserial-msgs ros-indigo-rosserial-python ros-indigo-ros-control ros-indigo-ros-controllers```

### ARM and AVR compilers and tools

`To be written: Install of arm-none-eabi, openocd, avr-gcc, avrdude and script for rosserial-arduino fix`

## Roberto Installation

`To be written: Catkin workspace initialization`

## Viewing Roberto in Rviz

**NOTE:** Rviz is **not** a simulator, but only a visualizer. This means that there is no physics engine in Rviz, but it is merely an easy way to see visualize how the robot behaves.

`roslaunch roberto_description roberto_rviz.launch`

## Simulating Roberto in Gazebo

To start the simulation in Gazebo, the syntax for now is:
`roslaunch roberto_gazebo <world file to be launched>.launch`

As an example, to spawn Roberto in a gasstation environment run:
`roslaunch roberto_gazebo roberto_world.launch`

In order to see how Gazebo is supposed to interact with real hardware and ROS, please see http://gazebosim.org/tutorials?tut=ros_control&cat=connect_ros