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


def get_taskplan2(goal):
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

    taskplan_server2.set_succeeded(result, "Task has been settled")


# task list1: store a sequence of (event, rid): dictionary
tasklist1_1 = ['b']
transList1_1 = [Transition('b',[3]),Transition('e',[3])]
robotlist1_1 = 3
subeventList1_1 = [['b']]
taskplan1_ = Taskplan(tasklist1_1, transList1_1, robotlist1_1, 1, subeventList1_1, 0)

# task list2: store a sequence of (event, rid): dictionary
tasklist2_1 = ['d']
transList2_1 = [Transition('d',[3]), Transition('e',[3])]
robotlist2_1 = 3
subeventList2_1 = [['d']]
taskplan2_ = Taskplan(tasklist2_1, transList2_1, robotlist2_1, 1, subeventList2_1, 1)

taskplans = [taskplan1_, taskplan2_]


try:
    rospy.init_node('taskplanning2', anonymous=True)

    taskplan_server2 = actionlib.SimpleActionServer('/taskplan1', PlanindexAction, get_taskplan2, False)
    taskplan_server2.start()

    rospy.spin()
except rospy.ROSInterruptException:
    pass


