#!/usr/bin/env python
from __future__ import print_function
import rospy

from six.moves import input

import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from human_baxter_collaboration.srv import Transformation
from human_baxter_collaboration.msg import BaxterTrajectory


class GripperCommander():

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node("FSM", anonymous=True)
        self.gripper = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "right_arm"
        #print(self.group_name)
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        #print("============ End effector link: %s" % self.eef_link)
        self.group_names = self.gripper.get_group_names()
        #print("============ Available Planning Groups:", self.group_names)
        #print("============ Printing robot state")
        self.box_name = ""
        #print(self.gripper.get_current_state())
        #print("")

    def add_table(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = 'world'
        # invertire coordinate
        box_pose.pose.position.x = 0.7673182
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = 0.4  
        box_name = 'world'
        scene.add_box(box_name, box_pose, size=( 2 ,0.6, 0.8))
        self.box_name = box_name
        


       

    def go_to_pose_goal(self, pose_goal):
        global pub
        # (self.move_group).set_start_state(*((self.move_group).getCurrentState()))
        (self.move_group).set_start_state(self.gripper.get_current_state())
        (self.move_group).set_goal_tolerance(0.5)
        (self.move_group).set_pose_target(pose_goal, self.eef_link)
        #(self.move_group).set_pose_target(pose_goal)
        plan = self.move_group.plan()
        msg = BaxterTrajectory()
        msg.trajectory = plan
        msg.arm = "right"
        pub.publish(msg)
        rospy.loginfo(plan)
        self.move_group.stop()

        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        

    def all_close(self, goal, actual, tolerance):

        if isinstance(goal, list):
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif isinstance(goal, geometry_msgs.msg.PoseStamped):
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif isinstance(goal, geometry_msgs.msg.Pose):
            return self.all_close(
                pose_to_list(goal),
                pose_to_list(actual),
                tolerance)

        return True


if __name__ == "__main__":
    global pub
    rospy.init_node("MoveIt_try")
    client_trans = rospy.ServiceProxy('transform', Transformation)
    trans = client_trans('C')
    pub = rospy.Publisher('/baxter_moveit_trajectory',
                                              BaxterTrajectory,
                                              queue_size=20)
    goal_pose = geometry_msgs.msg.Pose()
    goal_pose.position.x = trans.transform.transform.translation.x
    goal_pose.position.y = trans.transform.transform.translation.y
    goal_pose.position.z = trans.transform.transform.translation.z
    #goal_pose.orientation.x = trans.transform.transform.rotation.x
    #goal_pose.orientation.y = trans.transform.transform.rotation.y
    #goal_pose.orientation.z = trans.transform.transform.rotation.z
    #goal_pose.orientation.w = trans.transform.transform.rotation.w
   
    g_right = GripperCommander()
    #g_right.add_table()
    g_right.go_to_pose_goal(goal_pose)
    rospy.spin()
