#! /usr/bin/env python

import rospy
import actionlib
from mrs_smp.msg import TaskAction, TaskGoal, TaskResult
from mrs_taskplan.msg import PlanindexAction, PlanindexGoal, PlanindexResult, PlanindexFeedback
from mrs_planner_client_class import Transition, Taskplan

"""First, construct four clients for all the (four) robots"""
# #####################################################
global keph0_task_client, keph1_task_client, keph2_task_client, keph3_task_client
global kephera_clients


def get_taskplan(goal):
    # 0.1 feedback info
    feedback = PlanindexFeedback()
    feedback.group = goal.group

    # 0.2 Receiving task and goal info
    result = PlanindexResult()
    result.step = goal.step
    result.group = goal.group

    for taskplan in taskplans:
        if taskplan.planid == goal.step:
            taskplan.send_goal()

    taskplan_server1.set_succeeded(result, "Task has been settled")


# task list1: store a sequence of (event, rid): dictionary
tasklist1 = [('a', 'epslion'), ('c', 'epslion'), ('epslion', 'f')]
transList1 = [Transition('a',[1]), Transition('b',[1]), Transition('f',[2])]
#   Transition('d',[2]), Transition('e',[2]), Transition('f',[2]), \
#   Transition('g',[1,2])]
robotlist1 = (1,2)
subeventList1 = [['a', 'b', 'f']]
taskplan1 = Taskplan(tasklist1, transList1, robotlist1, 0, subeventList1, 0)

# task list2: store a sequence of (event, rid): dictionary
tasklist2_0 = ['f', 'e']
transList2_0 = [Transition('c',[2]), Transition('e',[2]), Transition('d',[2])]
robotlist2_0 = 2
subeventList2_0 = [['c', 'd', 'e']]
taskplan2 = Taskplan(tasklist2_0, transList2_0, robotlist2_0, 0, subeventList2_0, 1)

taskplans = [taskplan1, taskplan2]


try:
    rospy.init_node('taskplanning1', anonymous=True)

    taskplan_server1 = actionlib.SimpleActionServer('/taskplan0', PlanindexAction, get_taskplan, False)
    taskplan_server1.start()

    rospy.spin()
except rospy.ROSInterruptException:
    pass

