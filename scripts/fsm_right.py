#!/usr/bin/env python
"""
.. module:: fsm_right
    :platform: Unix
    :synopsis: Python module for implementing the finite state machine of the right gripper 

.. moduleauthor:: Giovanni Di Marco - imdimark@gmail.com
                  Alice Nardelli - alice.nardelli98@gmail.com
                  Federico Civetta - fedeunivers@gmail.com
                  Iacopo Pietrasanta - iacopo.pietrasanta@gmail.com

The ontology and the planner is done by the two finite state machines node.  We have decidedto implement a specific node for each gripper in order to manage them individually

Subscribes to:
     /blocks_state
     /right_gripper_pose

Publishes to:
    /baxter_movit_trajectory
    /open_close_right

Service :
     /occup_middleware 

Clients:
     /free_middleware
     /gl/trnasform


"""
from __future__ import print_function
import rospy

from six.moves import input
import numpy as np
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import *
from moveit_commander.conversions import pose_to_list
from human_baxter_collaboration.srv import Transformation
from human_baxter_collaboration.msg import BaxterTrajectory
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from human_baxter_collaboration.msg import BlocksState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import tf
from std_srvs.srv import *

def clbk_array(msg):
    """
    Description of the '' function:
           
    lore ipsum
           
    
     Args :
             None
    
    Returns :
             None

    """
    global blocks_array
    for i in range(5):
        blocks_array[i] = msg.blocksarray[i]


def clbk_ee(msg):
    """
    Description of the '' function:
           
    lore ipsum
           
    
     Args :
             None
    
    Returns :
             None

    """
    global ee
    ee = msg.pose

def clbk(req):
    """
    Description of the '' function:
           
    lore ipsum
           
    
     Args :
             None
    
    Returns :
             None

    """
    global middleware
    middleware = False
    return []
    
class GripperCommander():
    """
    This is a the class representing the implementation of the right Gripper commander

    param object: None
    :type object: class:`GripperCommander`
    :param name: scene
    :type name: Any
    :param name: group_name referring to the right arm group
    :type str
    :param name: move_group
    :type Any
    :param name: planning_frame
    :type Any
    :param name: eef_link
    :type Any
    :param name: group_names
    :type str
    :param name: box_name the name of the box 
    :type str
		
    """
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        self.gripper = moveit_commander.RobotCommander()
        rospy.sleep(1)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "right_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.planning_frame = self.move_group.get_planning_frame()

        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.gripper.get_group_names()
        self.box_name = ""

    def go_to_pose_goal(self, pose_goal):
        """
        Description of the '' function:
           
        lore ipsum
           
    
        Args :
             None
    
        Returns :
             None

        """
        global pub
        print(pose_goal)
        camera_info = rospy.wait_for_message(
            '/baxter_joint_states', JointState)
        array_states = [0, 0, 0, 0, 0, 0, 0]
        for i in range(1, 8):
            array_states[i - 1] = camera_info.position[i]

        joint_state = JointState()
        
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = [
            'right_s0',
            'right_s1',
            'right_e0',
            'right_e1',
            'right_w0',
            'right_w1',
            'right_w2']
        joint_state.position = array_states
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        (self.move_group).set_start_state(moveit_robot_state)
        (self.move_group).set_goal_tolerance(0.001)
        (self.move_group).set_pose_target(pose_goal)
        plan = self.move_group.plan()
        msg = BaxterTrajectory()
        msg.trajectory.append(plan[1])
        msg.arm = "right"
        pub.publish(msg)
        self.move_group.stop()

        self.move_group.clear_pose_targets()

    def add_table(self, timeout=4):
        """
        Description of the '' function:
           
        lore ipsum
           
    
        Args :
             None
    
        Returns :
             None

        """

        box_name = self.box_name
        scene = self.scene
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'world'
        # invertire coordinate
        box_pose.pose.position.x = 0
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = 0.4
        box_name = "Table"
        scene.add_box(box_name, box_pose, size=(2, 2, 0.75))
        box_pose.pose.position.x = 1.1
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = 1
        box_name = "Human"
        scene.add_box(box_name, box_pose, size=(0.1, 2, 2))

    def difference(self, a, b, threshold):
        """
        Description of the '' function:
           
        lore ipsum
           
    
        Args :
             None
    
        Returns :
             None

        """
        
        return np.abs(a - b) < threshold

    def fsm(self):
        """
        Description of the '' function:
           
        lore ipsum
           
    
        Args :
             None
    
        Returns :
             None

        """
        global  state, blocks_array, end, working, blocks_id
        global ee, selected_position
        global x_mid, y_mid, z_mid
        global x_goal_trans, y_goal_trans, z_goal_trans
        global pub_oc, middleware, client_mid
        if state == 0:  # rest
            x_goal_trans = 0
            y_goal_trans = 0
            if blocks_array[0] == 0 and blocks_array[1] == 0 and blocks_array[2] == 0 and blocks_array[3] == 0 and blocks_array[4] == 0:
                end = True
            else:
                for i in range(0, 5):
                    if blocks_array[i] == 1:
                        state = 1
        if state == 1:  # reaching
            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z
            if not working:
                print("stato1")
                msg_oc=Bool()
                msg_oc.data=False
                pub_oc.publish(msg_oc)
                for j in range(5):
                        if blocks_array[j] == 1:
                            selected_position = j
                            break
                
                
                goal_trans = client_trans(blocks_id[selected_position])
                print(goal_trans)
                x_goal_trans = goal_trans.transform.transform.translation.x
                y_goal_trans = goal_trans.transform.transform.translation.y
                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_goal_trans
                goal_pose.position.y = y_goal_trans
                goal_pose.position.z = 1.2
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                working=True
                self.go_to_pose_goal(goal_pose)
                
            else:

                if self.difference(
                        x_goal_trans,
                        x_ee,
                        0.01) & self.difference(
                        y_goal_trans,
                        y_ee,
                        0.01):
                        
                    print(x_ee)
                    print(y_ee)
                    print(z_ee)
                    state = 2
                    working = False
        if state == 2:  # discesa
            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z

            if not working:
                print("stato 2")
                goal_trans = client_trans(blocks_id[selected_position])
                x_goal_trans = goal_trans.transform.transform.translation.x
                y_goal_trans = goal_trans.transform.transform.translation.y
                z_goal_trans = goal_trans.transform.transform.translation.z
                

                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_goal_trans
                goal_pose.position.y = y_goal_trans
                goal_pose.position.z = z_goal_trans
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                working=True
                self.go_to_pose_goal(goal_pose)
                
            else:

                if self.difference(z_goal_trans, z_ee, 0.01):
                    print(x_ee)
                    print(y_ee)
                    print(z_ee)
                    state = 3
                    working = False
                    msg_oc=Bool()
                    msg_oc.data=True
                    pub_oc.publish(msg_oc)
                    
        if state == 3:  # sollevamento
            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z

            if not working:
                print('stato 3')
                

                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_ee
                goal_pose.position.y = y_ee
                goal_pose.position.z = 1
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                working=True
                self.go_to_pose_goal(goal_pose)
            else:

                if self.difference(1, z_ee, 0.01):
                    state = 4
                    working = False

        if state == 4:  # centro

            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z
            if not working:
                print('stato 4')
                box = client_trans('MiddlePlacementN')
                x_mid = box.transform.transform.translation.x
                y_mid = box.transform.transform.translation.y
                z_mid = box.transform.transform.translation.z
                

                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_mid
                goal_pose.position.y = y_mid
                goal_pose.position.z = 1.2
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                
                if not middleware:
                	working=True        
                	self.go_to_pose_goal(goal_pose)
            else:

                if self.difference(
                        x_mid,
                        x_ee,
                        0.01) & self.difference(
                        y_mid,
                        y_ee,
                        0.01):
                    state = 5
                    working = False
        if state == 5:  # discesa
            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z
            if not working:
                print('stato 5')
                box = client_trans('MiddlePlacementN')
                x_mid = box.transform.transform.translation.x
                y_mid = box.transform.transform.translation.y
                z_mid = box.transform.transform.translation.z

                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_mid
                goal_pose.position.y = y_mid
                goal_pose.position.z = z_mid + 0.1
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                working=True
                self.go_to_pose_goal(goal_pose)
            else:

                if self.difference( z_mid + 0.1 , z_ee, 0.01):

                    state = 6
                    working = False
                    msg_oc=Bool()
                    msg_oc.data=False
                    pub_oc.publish(msg_oc)
                    middleware = True
                    resp = client_mid()
         
        if state == 6:  # risalita
            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z
            if not working:
                print('stato 6')
             

                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x =0.74
                goal_pose.position.y =-0.335
                goal_pose.position.z = 1
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                working=True
                self.go_to_pose_goal(goal_pose)
            else:

                if self.difference(1, z_ee, 0.05):
                    state = 0
                    working = False


if __name__ == "__main__":
    global blocks_array, client_trans, state, working, end, selected_position
    global client_trans, pub_oc
    global x_mid, y_mid, z_mid
    global x_goal_trans, y_goal_trans, z_goal_trans, middleware
    rospy.init_node("fsm_right")
    blocks_array = [4, 4, 4, 4, 4]
    state = 0
    end = False
    working = False
    blocks_id = ['C', 'E', 'G', 'I', 'M']
    selected_position = 0
    x_goal_trans = 0
    y_goal_trans = 0
    z_goal_trans = 0
    x_mid = 0
    y_mid = 0
    z_mid = 0
    middleware = True
    s1 = rospy.Service('/free_middleware', Empty, clbk)
    client_mid = rospy.ServiceProxy('/occup_middleware', Empty)
    
    client_trans = rospy.ServiceProxy('/gr/transform', Transformation)
    sub_array = rospy.Subscriber("/blocks_state", BlocksState, clbk_array)
    sub = rospy.Subscriber("/right_gripper_pose", PoseStamped, clbk_ee)
    pub = rospy.Publisher('/baxter_moveit_trajectory',
                          BaxterTrajectory,
                          queue_size=20)
    pub_oc = rospy.Publisher('/open_close_right',
                          Bool,
                          queue_size=20)
    msg_oc=Bool()
    msg_oc.data=False
    pub_oc.publish(msg_oc)
    g_right = GripperCommander()
    g_right.add_table()
    rospy.sleep(1)
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        g_right.fsm()
        rate.sleep()
