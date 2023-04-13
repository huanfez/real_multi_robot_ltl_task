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

# turtlebot3 192.168.1.34
# ['/LED12', '/LED13', '/LED24', '/LED25']
# turtlebot3 192.168.1.37
# ['/LED17', '/LED16', '/LED2', '/LED3']
try:
    # geometry_name, agent_radius, agent_id, v_max, initial_position
    currentDir = '/home/i2rlab/catkin_ws/src/mrs_smp/node/results/'
    textfile_tb_d = open(currentDir + 'trajtbd.txt', 'w')
    textfile_tb_a = open(currentDir + 'trajtba.txt', 'w')
    task_pos_dict = {'a': [0.12,1.38], 'b': [1.28,2.76], 'c': [1.28,0.15], 'd': [2.58,2.76],\
               'e': [2.58,1.38], 'f': [2.58,0.12], 'g': [3.58,1.55]} #'c': [1.28,0.12]
#    robot2 = Turtlebot3('ColumbusCirclePoly', 0.08, 1, 0.18, [0.5, 2.5],\
#       '/burger37/cmd_vel', ['/LED17', '/LED16', '/LED2', '/LED3'], [textfile_tb_d, textfile_tb_a], task_pos_dict, 0.050, 0)
    robot2 = Turtlebot3('ColumbusCirclePoly', 0.08, 1, 0.15, [0.5, 2.5],\
        '/burger34/cmd_vel', ['/LED12', '/LED13', '/LED24', '/LED25'], [textfile_tb_d, textfile_tb_a], task_pos_dict, 0.050, 0)
    robot2.task_server.start()
    rospy.spin()
    textfile_tb_d.close()
    textfile_tb_a.close()
except rospy.ROSInterruptException:
    pass
