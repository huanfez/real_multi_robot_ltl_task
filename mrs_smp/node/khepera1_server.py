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


# robot 192.168.1.12
try:
    # geometry_name, agent_radius, agent_id, v_max, initial_position
    currentDir = '/home/i2rlab/catkin_ws/src/mrs_smp/node/results/'
    textfile12_d = open(currentDir + 'traj12d.txt', 'w')
    textfile12_a = open(currentDir + 'traj12a.txt', 'w')
    kepherarobot12 = KepRobot('ColumbusCirclePoly', 0.0442, 0, 0.15, [0.5, 0.5],\
       '/HKA/cmd_velGA', ['/LED14', '/LED15', '/LED18', '/LED19'], [textfile12_d, textfile12_a], 0.05, [])

    kepherarobot12.task_server.start()
    rospy.spin()
    textfile12_d.close()
    textfile12_a.close()
except rospy.ROSInterruptException:
    pass
