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


# robot 192.168.1.37
try:
    # geometry_name, agent_radius, agent_id, v_max, initial_position
    currentDir = '/home/i2rlab/catkin_ws/src/mrs_smp/node/results/'
    textfile37_d = open(currentDir + 'traj37d.txt', 'w')
    textfile37_a = open(currentDir + 'traj37a.txt', 'w')
    task_pos_dict = {'a': [0.12,1.38], 'b': [1.28,2.76], 'c': [1.28,0.2], 'd': [2.58,2.76],\
               'e': [2.58,1.38], 'f': [2.58,0.2], 'g': [3.48,1.38]} #'c': [1.28,0.12]

    turtlebot37 = Turtlebot3('ColumbusCirclePoly', 0.08, 2, 0.15, [3.5, 0.5],\
       '/burger37/cmd_vel', ['/LED17', '/LED16', '/LED2', '/LED3'], [textfile37_d, textfile37_a], task_pos_dict, 0.05, [2.38,1.38])

    turtlebot37.task_server.start()
    rospy.spin()
    textfile37_d.close()
    textfile37_a.close()
except rospy.ROSInterruptException:
    pass
