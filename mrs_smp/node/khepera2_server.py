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
    textfile10_d = open(currentDir + 'traj10d.txt', 'w')
    textfile10_a = open(currentDir + 'traj10a.txt', 'w')
    kepherarobot10 = KepRobot('ColumbusCirclePoly', 0.0442, 1, 0.20, [0.5, 2.5],\
       '/HKC/cmd_velGA', ['/LED10', '/LED11', '/LED8', '/LED9'], [textfile10_d, textfile10_a], 0.05, [])

    kepherarobot10.task_server.start()
    rospy.spin()
    textfile10_d.close()
    textfile10_a.close()
except rospy.ROSInterruptException:
    pass
