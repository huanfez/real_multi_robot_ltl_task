#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Vector3
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import numpy as np
import copy

import actionlib
from mrs_smp.msg import TaskAction, TaskGoal, TaskResult, TaskFeedback

import DiscretePlanner
import ContinousPlanner

from DiscretePlanner import HighLevelPlanner
from ContinousPlanner import Robot

from Turtlebot3_server_class import Turtlebot3


# robot 192.168.1.34
try:
    # geometry_name, agent_radius, agent_id, v_max, initial_position
    currentDir = '/home/i2rlab/catkin_ws/src/mrs_smp/node/results/'
    textfile34_d = open(currentDir + 'traj34d.txt', 'w')
    textfile34_a = open(currentDir + 'traj34a.txt', 'w')
    task_pos_dict = {'a': [0.12,1.38], 'b': [1.28,2.76], 'c': [1.28,0.2], 'd': [2.58,2.76],\
               'e': [2.48,1.38], 'f': [2.58,0.2], 'g': [3.58,1.38]} #case1:'e': [2.58,1.38] case2:'e': [2.48,1.38]

    turtlebot34 = Turtlebot3('ColumbusCirclePoly', 0.08, 3, 0.10, [3.5, 2.5],\
       '/burger34/cmd_vel', ['/LED12', '/LED13', '/LED24', '/LED25'], [textfile34_d, textfile34_a], task_pos_dict, 0.05, [])

    turtlebot34.task_server.start()
    rospy.spin()
    textfile34_d.close()
    textfile34_a.close()
except rospy.ROSInterruptException:
    pass
