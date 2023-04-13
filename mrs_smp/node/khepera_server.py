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

from Khepera_server_class import KepRobot

# robot 192.168.1.10
try:
    # geometry_name, agent_radius, agent_id, v_max, initial_position
    currentDir = '/home/i2rlab/catkin_ws/src/mrs_smp/node/results/'
    textfile2_d = open(currentDir + 'traj2d.txt', 'w')
    textfile2_a = open(currentDir + 'traj2a.txt', 'w')
    kepherarobot2 = KepRobot('ColumbusCirclePoly', 0.0442, 1, 0.20, [0.5, 2.5],\
       '/HKC/cmd_velGA', ['/LED10', '/LED11', '/LED8', '/LED9'], [textfile2_d, textfile2_a], 0.05, 0)
    kepherarobot2.task_server.start()
    rospy.spin()
    textfile2_d.close()
    textfile2_a.close()
except rospy.ROSInterruptException:
    pass
