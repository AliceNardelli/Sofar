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

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class GripperCommander():
    

    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node("FSM", anonymous=True)
        self.gripper = moveit_commander.RobotCommander() 
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "right_arm"
        print(self.group_name)
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.trajectory_pub = rospy.Publisher('/baxter_moveit_trajectory',
                                                   BaxterTrajectory,
                                                   queue_size=20)
        
        self.planning_frame= self.move_group.get_planning_frame()
       
        self.eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % self.eef_link)

        self.group_names = self.gripper.get_group_names()
        print("============ Available Planning Groups:", self.group_names)

        
        print("============ Printing robot state")
      
        print(self.gripper.get_current_state())
        print("")    
     
     


    def go_to_pose_goal(self, pose_goal):      
        
    
        (self.move_group).set_pose_target(pose_goal,)
        #plan+execute
        
        plan = self.move_group.plan()
        self.trajectory_pub.publish(plan)
        rospy.loginfo (plan)
        self.move_group.stop()
        
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


if __name__ == "__main__":
        
        rospy.init_node("FSM")
        client_trans= rospy.ServiceProxy('transform', Transformation)
        blue_box = geometry_msgs.msg.TransformStamped()
        blue_box=client_trans('BlueBox')
        box=['E','C','I','G','M']
        t=client_trans('A')
        goal_pose = geometry_msgs.msg.Pose() 
        goal_pose.position.x = 3
        goal_pose.position.y = 3
        goal_pose.position.z = 3
        goal_pose.orientation.w = 3
        #g_left = GripperCommander("left_arm")
        g_right = GripperCommander()
        #g_left.go_to_pose_goal(goal_pose)

        g_right.go_to_pose_goal(goal_pose)
        rospy.spin()

