#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Vector3
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import numpy as np
import copy

import actionlib
from mrs_smp.msg import TaskAction, TaskGoal, TaskResult, TaskFeedback

from mrs_taskplan.msg import Trajectory

import DiscretePlanner
import ContinousPlanner

from DiscretePlanner import HighLevelPlanner
from ContinousPlanner import Robot

PI = 3.1415927
currentDir = '/home/i2rlab/catkin_ws/src/mrs_smp/node/'

class Turtlebot3():
    #def __init__(self, geometry_name, agent_radius, agent_id, v_max, initial_position, time_step=0.05, temp_obstacle = 0):
    def __init__(self, geometry_name, agent_radius, agent_id, \
       v_max, initial_position, velpuber_name, leds, textfiles, \
       task_pos_dict, time_step=0.05, temp_obstacle = []):

        node_name = 'robot'+ str(agent_id) + '_server'
        rospy.init_node(node_name, anonymous=True)

        server_name = 'kephera' + str(agent_id) + 'task'
        self.task_server = actionlib.SimpleActionServer(server_name, TaskAction, self.get_taskstate, False)
        self.hplanner = HighLevelPlanner(geometry_name, initial_position, agent_id, temp_obstacle)

        self.geometry_name = geometry_name
        self.agent_radius = agent_radius
        self.agent_id = agent_id
        self.v_max = v_max
        self.time_step = time_step

        self.ledf = Vector3() # Front LED
        self.ledb = Vector3() # Back LED
        self.ledl = Vector3() # Left LED
        self.ledr = Vector3() # Right LED
        #'/HKC/cmd_velGA' '/LED10' '/LED11' '/LED8' '/LED9'
        self.vel_publisher = rospy.Publisher(velpuber_name, Twist, queue_size = 10)
        # trajectory publisher
        self.dTrajectory_publisher = rospy.Publisher('dtraj_puber'+str(agent_id), Trajectory, queue_size=10)

        #self.kiii_pos_subscriber = rospy.Subscriber('HKC/odom', Odometry, self.get_odompos)
        self.posf_subscriber = rospy.Subscriber(leds[0], Vector3, self.get_posf)
        self.posb_subscriber = rospy.Subscriber(leds[1], Vector3, self.get_posb)
        self.posl_subscriber = rospy.Subscriber(leds[2], Vector3, self.get_posl)
        self.posr_subscriber = rospy.Subscriber(leds[3], Vector3, self.get_posr)

        self.twod_pose = initial_position
        self.actualpos6 = Pose() # actual position in 6 dimensions
        self.taskpos6 = Pose() # task coordinates
        self.trajectory = [self.twod_pose]
        self.vel_list = []

        self.rate = rospy.Rate(20)
        self.task_pos_dict = task_pos_dict
        self.textfile_d = textfiles[0]
        self.textfile_a = textfiles[1]


    """ Call back function: based on if the task has been reached,
    get newly assigned task"""
    def get_taskstate(self, goal):

        desiredTraj = Trajectory()

        # 0.1 feedback info
        feedback = TaskFeedback()
        feedback.task = goal.task
        feedback.rid = goal.rid
        feedback.state = False

        # 0.2 Receiving task and goal info
        result = TaskResult()
        result.task = goal.task
        result.rid = goal.rid
        result.teamid = goal.teamid
        result.state = False

        replan = False
        #self.cplanner.is_finished = False

        # 1.1 discrete planner
        self.hplanner.goals = np.array(self.task2goal_pose(goal.task))

        if goal.task == 'g' and goal.rid == 2:
            self.hplanner.temp_obstacle_pos = [3.38,1.38]
        elif goal.task == 'g' and goal.rid == 3:
            self.hplanner.temp_obstacle_pos = [3.48,1.38]

        self.hplanner.initial_position = self.twod_pose
        self.hplanner.discrete_planner()
        print 'receive goal task: ' + goal.task + \
           'Translated goal num: ' + str(self.task2goal_state(goal.task)) + \
           'goal triangle num: ' + str(self.hplanner.goal_triangle_num)

        # 1.2 continous planner
        self.cplanner = Robot(self.geometry_name, self.agent_radius, \
           self.agent_id, self.v_max, self.twod_pose, self.time_step)

        # 1.3 generate trajectory and velocity
        self.update_pose()
        self.trajectory = [[self.twod_pose[0], self.twod_pose[1], self.actualpos6.theta]]
        self.vel_list = []
        #previous_twod_vel = np.array([0.0, 0.0])
        while not self.cplanner.is_finished:
            # generate the two dimensional velocity
            current_twod_vel = self.cplanner.move(self.twod_pose)
            abs_current_twod_vel = sqrt(current_twod_vel[0]**2 + current_twod_vel[1]**2)
            if abs_current_twod_vel < 0.001:
                current_twod_vel = current_twod_vel / abs_current_twod_vel * 0.02
                abs_current_twod_vel = sqrt(current_twod_vel[0]**2 + current_twod_vel[1]**2)

            # calaulate and record the reference position
            pose_xy = np.array(self.twod_pose) + self.time_step * current_twod_vel
            self.twod_pose = [pose_xy[0] , pose_xy[1]]
            roll = atan2(current_twod_vel[1], current_twod_vel[0])
            self.trajectory.append([self.twod_pose[0], self.twod_pose[1], roll])
            self.textfile_d.write(str(self.twod_pose[0]) + ' ' + str(self.twod_pose[1]) + '\n')

            # calaulate and record the reference velocity
            roll_vel = (self.trajectory[-1][2] - self.trajectory[-2][2]) / self.time_step
            self.vel_list.append([current_twod_vel[0], current_twod_vel[1], abs_current_twod_vel, roll_vel])

        desiredTraj.xlist = [item[0] for item in self.trajectory]
        desiredTraj.ylist = [item[1] for item in self.trajectory]

        self.dTrajectory_publisher.publish(desiredTraj)

        """ use robot position subscriber to get autual position info self.actualpos6
        if not reaching, feedback some information about task
        else tell the taskplan node: the task has been completed"""
        print('current position is: ', self.actualpos6.x, self.actualpos6.y, self.actualpos6.theta)
        print len(self.vel_list)

        # 2.1 Rotating a angle
        initial_direction = atan2(self.vel_list[0][1], self.vel_list[0][0])
        self.rotate(initial_direction)

        # 2.2 following a trajectory
        step = 0
        while not result.state:
            self.task_server.publish_feedback(feedback)

            """when potential collision happens, replan both discrete and co
            ntinous path"""
            if self.cplanner.replan_request == True:
                print 'replanned'

            # Generate and send velocity info.
            desired_pose = self.trajectory[step+1]
            desired_vel = self.vel_list[step]

            self.update_pose()
            err_px = desired_pose[0] - self.actualpos6.x
            err_py = desired_pose[1] - self.actualpos6.y
            err_theta = desired_pose[2] - self.actualpos6.theta
            errors = np.array([[err_px], [err_py], [err_theta]])

            #while (errors[0][0]**2 + errors[1][0]**2) > 0.001 or errors[3][0] > 0.1:
            ctrl_velocity = self.ctrl2_vel(desired_pose, desired_vel)
            self.textfile_a.write(str(self.actualpos6.x) + ' ' + str(self.actualpos6.y) + '\n')

            self.vel_publisher.publish(ctrl_velocity)
            self.rate.sleep()
            #self.update_pose()
            step += 1

            # result info
            if step > len(self.vel_list) - 1:
                print "succeed"
                result.state = True

        # 3. Last step for stoping the motion
        ctrl_velocity = Twist()
        self.vel_publisher.publish(ctrl_velocity)
        self.task_server.set_succeeded(result, "Task has been settled")


    """use dictionary to find the position value of event:label"""
    def task2goal_pose(self, task):
        return self.task_pos_dict[task]


    """use dictionary to find the state value of event:label"""
    def task2goal_state(self, task):
        goal_pose = self.task_pos_dict[task]
        print "goal pose: " + str(goal_pose)
        return self.hplanner.pose2discrete_state(np.array(goal_pose))


    ''' Front LED position info '''
    def get_posf(self, pose1):
        self.ledf.x = pose1.x
        self.ledf.y = pose1.y
        self.ledf.z = pose1.z


    ''' Back LED position info '''
    def get_posb(self, pose2):
        self.ledb.x = pose2.x
        self.ledb.y = pose2.y
        self.ledb.z = pose2.z


    ''' Left LED position info '''
    def get_posl(self, pose3):
        self.ledl.x = pose3.x
        self.ledl.y = pose3.y
        self.ledl.z = pose3.z


    ''' Right LED position info '''
    def get_posr(self, pose4):
        self.ledr.x = pose4.x
        self.ledr.y = pose4.y
        self.ledr.z = pose4.z


    ''' Estimate robot position based on 4 LEDs position info '''
    def update_pose(self):
        ledf_flag = True
        ledb_flag = True
        ledl_flag = True
        ledr_flag = True

        if self.ledf.x == 0 and self.ledf.y == 0 and self.ledf.z == 0:
            ledf_flag = False

        if self.ledb.x == 0 and self.ledb.y == 0 and self.ledb.z == 0:
            ledb_flag = False

        if self.ledl.x == 0 and self.ledl.y == 0 and self.ledl.z == 0:
            ledl_flag = False

        if self.ledr.x == 0 and self.ledr.y == 0 and self.ledr.z == 0:
            ledr_flag = False

        if ledf_flag and ledb_flag and ledl_flag and ledr_flag:
            self.actualpos6.y = (self.ledf.x + self.ledb.x + self.ledl.x + self.ledr.x) / 4.0 /1000.0 + 1.45
            self.actualpos6.x = -(self.ledf.y + self.ledb.y + self.ledl.y + self.ledr.y) / 4.0 /1000.0 + 1.83
            diff_fb_y = self.ledf.x - self.ledb.x
            diff_fb_x = -(self.ledf.y - self.ledb.y)
            diff_lr_y = self.ledl.x - self.ledr.x
            diff_lr_x = -(self.ledl.y - self.ledr.y)
            self.actualpos6.theta = (atan2(diff_fb_y, diff_fb_x) + atan2(diff_lr_y, diff_lr_x) - PI/2.0) / 2.0
        elif ledf_flag and ledb_flag and not ledl_flag and not ledr_flag:
            self.actualpos6.y = (self.ledf.x + self.ledb.x) / 2.0 / 1000.0 + 1.45
            self.actualpos6.x = -(self.ledf.y + self.ledb.y) / 2.0 / 1000.0 + 1.83
            diff_fb_y = self.ledf.x - self.ledb.x
            diff_fb_x = -(self.ledf.y - self.ledb.y)
            self.actualpos6.theta = atan2(diff_fb_y, diff_fb_x)
        elif not ledf_flag and not ledb_flag and ledl_flag and ledr_flag:
            self.actualpos6.y = (self.ledl.x + self.ledr.x) / 2.0 / 1000.0 + 1.45
            self.actualpos6.x = -(self.ledl.y + self.ledr.y) / 2.0 / 1000.0 + 1.83
            diff_lr_y = self.ledl.x - self.ledr.x
            diff_lr_x = -(self.ledl.y - self.ledr.y)
            self.actualpos6.theta = atan2(diff_lr_y, diff_lr_x) - PI/2.0


    # Use controller to generate the velocity
    def ctrl1_vel(self, desired_position, desired_velocity):
        cmd_vel = Twist()

        self.update_pose()
        err_px = desired_position[0] - self.actualpos6.x
        err_py = desired_position[1] - self.actualpos6.y
        err_theta = desired_position[2] - self.actualpos6.theta
        errors = np.array([[err_px], [err_py], [err_theta]])

        matrix = np.array([[np.cos(self.actualpos6.theta), np.sin(self.actualpos6.theta), 0],\
                           [-np.sin(self.actualpos6.theta), np.cos(self.actualpos6.theta), 0],\
                           [0, 0, 1]])
        error_array_ = matrix.dot(errors)
        #print error_array_

        # coefficients
        b = 0.01
        epsilon = 0.03
        a = sqrt(desired_velocity[3]**2 + b * (desired_velocity[2]**2))

        K_x = 2 * epsilon * a
        K_y = b * desired_velocity[2]
        K_the = K_x

        cmd_vel.linear.x = desired_velocity[2] * np.cos(error_array_[2][0]) + K_x * error_array_[0][0]
        cmd_vel.angular.z = desired_velocity[3]+ (K_y * error_array_[1][0] * np.sin(error_array_[2][0]) / error_array_[2][0]) + K_the * error_array_[2][0]
        #print 'velocity:' + str(cmd_vel.linear.x) + ',' + str(cmd_vel.angular.z) + ',' + str(theta1 - self.actualpos6.theta)
        return cmd_vel


    # Use controller to generate the velocity
    def ctrl2_vel(self, desired_position, desired_velocity):
        cmd_vel = Twist()
#        kx = 0.8 # 2.0 for robot 192.168.1.13
#        ky = 0.8 # 2.0
#        b = 0.2 # 0.1

        kx = 0.1 # 2.0 for robot 192.168.1.13
        ky = 0.1 # 2.0
        b = 0.09 # 0.1

        self.update_pose()
        err_px = desired_position[0] - self.actualpos6.x
        err_py = desired_position[1] - self.actualpos6.y
        err_theta = desired_position[2] - self.actualpos6.theta
        errors = np.array([[err_px], [err_py], [err_theta]])

        desired_velocity[0] += kx * err_px
        desired_velocity[1] += ky * err_py
        cmd_vel.linear.x = desired_velocity[0] * np.cos(self.actualpos6.theta) + desired_velocity[1] * np.sin(self.actualpos6.theta)
        cmd_vel.angular.z = 1.0 / b * (desired_velocity[1] * np.cos(self.actualpos6.theta) - desired_velocity[0] * np.sin(self.actualpos6.theta))
        #print 'velocity:' + str(cmd_vel.linear.x) + ',' + str(cmd_vel.angular.z) + ',' + str(theta1 - self.actualpos6.theta)
        return cmd_vel


    ''' Rotate first at the beginning of a task to adjust the angle'''
    def rotate(self, desired_angle):
        rot_vel = Twist()
        ka_z = 0.3

        self.update_pose()
        err_anglez = desired_angle - self.actualpos6.theta

        while abs(err_anglez) > 0.08:
            print 'angle error: ', err_anglez, desired_angle, self.actualpos6.x, self.actualpos6.y, self.actualpos6.theta
            rot_vel.angular.z = ka_z * err_anglez

            if rot_vel.angular.z > 0.6:
                rot_vel.angular.z = 0.6
            elif rot_vel.angular.z < -0.6:
                rot_vel.angular.z = -0.6

            if 0 < rot_vel.angular.z < 0.08:
                rot_vel.angular.z = 0.08
            elif -0.08 < rot_vel.angular.z < 0:
                 rot_vel.angular.z = -0.08

            self.vel_publisher.publish(rot_vel)
            #print 'rotation velocity: ', rot_vel.angular.z

            self.rate.sleep()
            self.update_pose()
            #print 'position: ', self.actualpos6.x, self.actualpos6.y, self.actualpos6.theta
            err_anglez = desired_angle - self.actualpos6.theta


