#!/usr/bin/env python

"""
.. module:: fsm_left
    :platform: Unix
    :synopsis: Python module for implementing the finite state machine of the left gripper 

.. moduleauthor:: Giovanni Di Marco - imdimark@gmail.com
                  Alice Nardelli - alice.nardelli98@gmail.com
                  Federico Civetta - fedeunivers@gmail.com
                  Iacopo Pietrasanta - iacopo.pietrasanta@gmail.com

The ontology and the planner is done by the two finite state machines node.  We have decidedto implement a specific node for each gripper in order to manage them individually

Subscribes to:
     /blocks_state
     /left_gripper_pose
     /baxter_joint_states

Publishes to:
    /baxter_movit_trajectory
    /open_close_left

Service servers :
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
    Description of the clbk_array function:
           
    continuously update the array in which are contained informations about blocks state
           
    
    Args :
      msg(int64[]) the blocksarray field of BlocksState.msg
    
    Returns :
      None
    """
    global blocks_array
    for i in range(5):
        blocks_array[i] = msg.blocksarray[i]


def clbk_ee(msg):
    """
    Description of the clbk_ee function:
           
    Read the actual position of the end effector of the left gripper, and copy subscribed value inside a global variable.
           
    
    Args :
      msg(PoseStamped)
    
    Returns :
      None
    """
    global ee
    ee = msg.pose
    
def clbk(req):
    """
    Description of the clbk function:
           
    This function is the callback of the /occup_middleware server. 
    Once the empty message is receive from client it means that the middle of the table is occupied.
    middleware variable is set to true
           
    
    Args :
      req(Empty)
    
    Returns :
      None

    """
    global middleware
    middleware=True
    return []

class GripperCommander():
    """
    This is a the class representing the implementation of the Gripper commander

    param object: None
    :type object: class:`GripperCommander`
    :param name: scene
    :type name: Any
    :param name: group_name referring to the left arm group
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
        self.group_name = "left_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.planning_frame = self.move_group.get_planning_frame()

        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.gripper.get_group_names()
        self.box_name = ""

    def go_to_pose_goal(self, pose_goal):
        """
        Description of the go_to_pose_goal function:
           
        This function is used to ask to Moveit to generated a plan for reaching a goal position.
        Once the plan is retrived this is published on /baxter_movit_trajectory to make execute it
           
    
        Args :
             pose_goal(Pose) is the position that the end effector should reaches
    
        Returns :
             None

        """
        global pub
        #single subscrition to get the state of Baxter joints
        camera_info = rospy.wait_for_message(
            '/baxter_joint_states', JointState)
        array_states = [0, 0, 0, 0, 0, 0, 0]
        for i in range(8, 15):
            array_states[i - 8] = camera_info.position[i]

        joint_state = JointState()
        
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = [
            'left_s0',
            'left_s1',
            'left_e0',
            'left_e1',
            'left_w0',
            'left_w1',
            'left_w2']
        joint_state.position = array_states
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        #gripper actual state is set as starting state
        (self.move_group).set_start_state(moveit_robot_state)
        #set a tolerance of 1mm
        (self.move_group).set_goal_tolerance(0.001)
        #set the goal pose as target
        (self.move_group).set_pose_target(pose_goal)
        #ask for a plan
        plan = self.move_group.plan()
        #fill the fields of BaxterTrajectory.msg
        msg = BaxterTrajectory()
        msg.trajectory.append(plan[1])
        msg.arm = "left"
        #publish the plan to make execute it
        pub.publish(msg)
        self.move_group.stop()

        self.move_group.clear_pose_targets()

    def add_table(self, timeout=4):
        """
        Description of the add_table function:
           
        This method is used to add to the scene fixed obstacles
           
    
        Args :
             timeout
    
        Returns :
             None

        """
        box_name = self.box_name
        scene = self.scene
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'world'
        # add the table
        #chose position where locate the obstacle
        box_pose.pose.position.x = 0
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = 0.4
        box_name = "Table"
        #add the box chosing suitable dimensions
        scene.add_box(box_name, box_pose, size=(2, 2, 0.75))
        #add the box in order to avoid collision with human body
        #set the pose
        box_pose.pose.position.x = 1.1
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = 1
        box_name = "Human"
        #add to the scene
        scene.add_box(box_name, box_pose, size=(0.1, 2, 2))


    def difference(self, a, b, threshold):
        """
        Description of the difference function:
           
        return true if the distance is smaller then the threshold
           
    
        Args :
             a(float64)
             b(float64)
             threshold(float64)
    
        Returns :
             bool
             

        """
        
        
        return np.abs(a - b) < threshold

    def fsm(self):
        """
        Description of the fsm function:
           
        In these function is contained the finite state machine of the node.
           
    
        Args :
             None
    
        Returns :
             None

        """
        global x_goal_trans, y_goal_trans, state, blocks_array, end, working, blocks_id
        global ee, selected_position
        global x_box, y_box, z_box
        global x_goal_trans, y_goal_trans, z_goal_trans, pub_oc, middleware , client_mid
       
        if state == 0:  #REST STATE
            x_goal_trans = 0
            y_goal_trans = 0
            #if all blocks are inside the box then end variable becomes true
            if blocks_array[0] == 0 and blocks_array[1] == 0 and blocks_array[2] == 0 and blocks_array[3] == 0 and blocks_array[4] == 0:
                end = True
            #on the other case is contntinuously checked the blocksarray to see if there is a blcks to grasp    
            else:
                #if there is a box at left or at the centre the state become 1
                for i in range(0, 5):
                    if blocks_array[i] == 2 or blocks_array[i] == 3:
                        state = 1
        if state == 1:  # REACHING STATE: the robot reack the blocks to grasp
            
            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z
            if not working: #if the robot is not moving
                
               
                rospy.sleep(4)
                if middleware: #middleware is set to true if there is a block at the centre
                #In that case blocks at the centre has priority
                #find which block is at the centre
                    for j in range(5):
                        if blocks_array[j] == 2:
                            selected_position = j
                            
                            break
                else:  #if there are not blocks at the centre find the first available node in the blocksarray
                    for j in range(5):
                        if blocks_array[j] == 3:
                            selected_position = j
                            break
        
                #get the transformation of the block to reach
                goal_trans = client_trans(blocks_id[selected_position])
                #set the goal to reach
                x_goal_trans = goal_trans.transform.transform.translation.x
                y_goal_trans = goal_trans.transform.transform.translation.y
                #x and y coordinates of the goal are the same of the block
                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_goal_trans
                goal_pose.position.y = y_goal_trans
                goal_pose.position.z = z_ee
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                #call the method for plan and make execute the goal
                self.go_to_pose_goal(goal_pose)
                working = True
            else:
            #if the gripper is working

                #monitor the distance with the goal
                if self.difference(
                        x_goal_trans,
                        x_ee,
                        0.01) & self.difference(
                        y_goal_trans,
                        y_ee,
                        0.01):
                    #if the goal has been reached
                    #the state is updated
                    state = 2
                    working = False
                    #robot ee is opened
                    msg_oc=Bool()
                    msg_oc.data=False
                    pub_oc.publish(msg_oc)
                    
        if state == 2:  # DOWNHILL STATE: the robot reaches the goal
            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z

            if not working:
                #get the transformation of the selected blocks
                goal_trans = client_trans(blocks_id[selected_position])
                x_goal_trans = goal_trans.transform.transform.translation.x
                y_goal_trans = goal_trans.transform.transform.translation.y
                z_goal_trans = goal_trans.transform.transform.translation.z
                
                #the goal pose is the same of the block
                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_goal_trans
                goal_pose.position.y = y_goal_trans
                goal_pose.position.z = z_goal_trans
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                #plan and execute
                self.go_to_pose_goal(goal_pose)
                working = True
            #if the gripper is working
            else:
                #monitor is on z coordinate of the end effector that must reach the heigth of the block
                if self.difference(z_goal_trans, z_ee, 0.01):

                    state = 3
                    working = False
                    #grasp the block
                    msg_oc=Bool()
                    msg_oc.data=True
                    pub_oc.publish(msg_oc)
                    
        if state == 3:  #ASCENT STATE
            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z
         
            if not working:
                
                #the gripper must go a movement of ascent along the z axis
                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_ee
                goal_pose.position.y = y_ee
                goal_pose.position.z = 1
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                
                self.go_to_pose_goal(goal_pose)
                working = True
            else:
                #monitor again only the z coordinates
                if self.difference(1, z_ee, 0.01):

                    state = 4
                    working = False
                    #middleware is set to false
                    middleware = False
                    resp = client_mid()
                    
        if state == 4:  #REACHING THE BLUE BOX

            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z
            if not working:
                #get the transform of the blue box
                box = client_trans('Bluebox')
                x_box = box.transform.transform.translation.x
                y_box = box.transform.transform.translation.y
                z_box = box.transform.transform.translation.z
                #goal pose has same x and y coordinates of the blue-box
                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_box
                goal_pose.position.y = y_box
                goal_pose.position.z = 1.2
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                self.go_to_pose_goal(goal_pose)
                working = True
            else:
                #monitor x and y coordinates of the end effector,
                # when they are the same of the box the goal is considered reached
                if self.difference(
                        x_box,
                        x_ee,
                        0.01) & self.difference(
                        y_box,
                        y_ee,
                        0.01):

                    state = 5
                    working = False
        if state == 5:  # DOWNHILL STATE
            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z
            if not working:
                #the goal pose is above the blue box
                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_ee
                goal_pose.position.y = y_ee
                goal_pose.position.z = z_box+0.35
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                self.go_to_pose_goal(goal_pose)
                working = True
            else:
                #when a certain height is reached the block is released
                if self.difference(z_box+0.35, z_ee, 0.01):
                    #state return to rest
                    state = 0
                    working = False
                    msg_oc=Bool()
                    msg_oc.data=False
                    
                    pub_oc.publish(msg_oc)
                    

if __name__ == "__main__":
    
    global blocks_array, client_trans, state, working, end, selected_position
    global client_trans, pub_oc
    global x_box, y_box, z_box
    global x_goal_trans, y_goal_trans, z_goal_trans, middleware
    
    
    rospy.init_node("fsm_left")
    
    #initialisation of needed variables
    blocks_array = [4, 4, 4, 4, 4]
    state = 0
    end = False
    working = False
    blocks_id = ['C', 'E', 'G', 'I', 'M']
    selected_position = 0
    x_goal_trans = 0
    y_goal_trans = 0
    z_goal_trans = 0
    x_box = 0
    y_box = 0
    z_box = 0
    middleware = False
    
    
    s1 = rospy.Service('/occup_middleware', Empty, clbk)
    client_mid = rospy.ServiceProxy('/free_middleware',  Empty)
    
    client_trans = rospy.ServiceProxy('/gl/transform', Transformation)
    sub_array = rospy.Subscriber("/blocks_state", BlocksState, clbk_array)
    sub = rospy.Subscriber("/left_gripper_pose", PoseStamped, clbk_ee)
    pub = rospy.Publisher('/baxter_moveit_trajectory',
                          BaxterTrajectory,
                          queue_size=20)
    pub_oc = rospy.Publisher('/open_close_left',
                          Bool,
                          queue_size=20)
    #OBJECT of the class GripperCommander()               
    g_left = GripperCommander()
    rospy.sleep(1)
    #set obstacles
    g_left.add_table()
    msg_oc=Bool()
    msg_oc.data=False
    pub_oc.publish(msg_oc)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        g_left.fsm()
        rate.sleep()
