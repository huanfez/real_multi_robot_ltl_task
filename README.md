# Lab multi-robot experiment achieving task plan synthesized from "parallel decomposition and concurrent satisfaction of LTL task specification"

this project runs real robots but does not have scalability computation experiments.

Ubuntu14.0 

ROS Indigo 

Python2

JAVA 6.0


***
This project contains the task planning package (automaton-based) with JAVA; and motion planning package (Turtlebot3, Khepera II) with ROS. 
Each has a folder, i.e., "TaskAllocAut" and "mrs_smp". Each folder contains its own read me files. `Please explore into each foler for the readme.md` files.


## Package1. TaskAllocAut (ouput task plan in a string format)
Explore into `/TaskAllocAut/src/taskPlanning2` subfolder for detailed explaination of operation


## Package2. mrs_smp (refer to the results from Package1 and conduct motion control with real robots)
Explore into `/mrs_smp` subfolder for detailed explaination of operation

Note outputs of two packages are separate and cannot automatically connnect with each other.

