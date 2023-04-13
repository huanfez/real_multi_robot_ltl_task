# Turtlebot3 and Khepera II ROS

Ubuntu14.0 

ROS Indigo 

Python2

Note: this package is to launch and control real robots in lab environment with ROS. All the operations can be directly conducted at the terminal. The lab directory of the file is `~/catkin_ws/src/mrs_smp/`

***
## Step1. configure the hardware system

_**Start the projector system**_ 

_**Start the phasespace system**_ 

_**Start the Khepera II robots**_ Just need to open the button. The two used robots are the one with ip `192.168.1.10` and the one with ip `ip 192.168.1.12` (may change)


_**Start the Tutorbot3 robots**_ First need to open the power button of the robot and run `roscore` at PC terminal. Then, use `ssh pi@{IP_ADDRESS_OF_RASPBERRY_PI}` to enter into each Tutorbot3's terminal (password is `turtlebot`). Finally, run `roslaunch turtlebot3_bringup turtlebot3_robot.launch` at turtlebot3 (raspberry pi) to fully start the robot. (More detailed startup infromation of Tutorbot3 can be found in official demo here: `https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup`)

***
## Step2. Launch the Kephera II robots and Phasespace system 

`$roslaunch mrs_smp all_robots_start.launch` 

***
## Step3. Start the motion planning and control of Kephera II robots and Turtlebot3 robots

`$roslaunch mrs_smp all_robots_smp.launch` 
_**start the symbolic motion planner of the multi-robot system, and also plots the realtime trajectory of each robot**_

