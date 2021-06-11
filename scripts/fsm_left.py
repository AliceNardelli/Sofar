#!/usr/bin/env python
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
from std_msgs.msg import String,Bool
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


def clbk_array(msg):
    global blocks_array
    for i in range(5):
        blocks_array[i] = msg.blocksarray[i]


def clbk_ee(msg):
    global ee
    ee = msg.pose
    
def clbk(msg):
    global middleware
    middleware=True


class GripperCommander():

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
        global pub
        print(pose_goal)
        camera_info = rospy.wait_for_message(
            '/baxter_joint_states', JointState)
        array_states = [0, 0, 0, 0, 0, 0, 0]
        for i in range(8, 15):
            array_states[i - 8] = camera_info.position[i]

        joint_state = JointState()
        # joint_state.header = Header()
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
        (self.move_group).set_start_state(moveit_robot_state)
        (self.move_group).set_goal_tolerance(0.001)
        (self.move_group).set_pose_target(pose_goal)
        plan = self.move_group.plan()
        msg = BaxterTrajectory()
        msg.trajectory.append(plan[1])
        msg.arm = "left"
        pub.publish(msg)
        self.move_group.stop()

        self.move_group.clear_pose_targets()

    def add_table(self, timeout=4):
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

    def add_cylinder(self, f_id, pose_obs, timeout=4):
        box_name = self.box_name
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = f_id
        box_pose.pose.position.x = pose_obs.transform.transform.translation.x
        box_pose.pose.position.y = pose_obs.transform.transform.translation.y
        box_pose.pose.position.z = pose_obs.transform.transform.translation.z
        box_name = f_id
        scene.add_box(box_name, box_pose,size=(0.1, 0.3, 0.1))
        self.box_name = box_name

    def remove_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        scene.remove_world_object(box_name)

    def human_collision(self):
        global working, client_trans
        trans_right = client_trans('lowerarm_r')
        trans_left = client_trans('lowerarm_l')
        self.add_cylinder('lowerarm_r', trans_right)
        self.add_cylinder('lowerarm_l', trans_left)
        working = True

    def difference(self, a, b, threshold):
        #rospy.loginfo(np.abs(a - b))
        return np.abs(a - b) < threshold

    def fsm(self):
        global x_goal_trans, y_goal_trans, state, blocks_array, end, working, blocks_id
        global ee, selected_position
        global x_box, y_box, z_box
        global x_goal_trans, y_goal_trans, z_goal_trans, pub_oc
        
        if state == 0:  # rest
            x_goal_trans = 0
            y_goal_trans = 0
            if blocks_array[0] == 0 and blocks_array[1] == 0 and blocks_array[2] == 0 and blocks_array[3] == 0 and blocks_array[4] == 0:
                end = True
            else:
                for i in range(0, 5):
                    if blocks_array[i] == 2 or blocks_array[i] == 3:
                        state = 1
        if state == 1:  # reaching
            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z
            if not working:
                print("stato1")
                #middleware = rospy.get_param('middleware')
                #middleware = False
                #if middleware:
                    #for j in range(5):
                        #if blocks_array[j] == 2:
                            #selected_position = j
                            #break
                #else:  # mettiamo i blocchi in modo tale che i primi a essere presi sono quelli a destra?
                for j in range(5):
                        if blocks_array[j] == 2:
                            selected_position = j
                            break
                        elif blocks_array[j] == 3:
                            selected_position = j
                            break

                self.human_collision()
                goal_trans = client_trans(blocks_id[selected_position])
                x_goal_trans = goal_trans.transform.transform.translation.x
                y_goal_trans = goal_trans.transform.transform.translation.y
                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_goal_trans
                goal_pose.position.y = y_goal_trans
                goal_pose.position.z = z_ee
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                self.go_to_pose_goal(goal_pose)

            else:

           
                if self.difference(
                        x_goal_trans,
                        x_ee,
                        0.01) & self.difference(
                        y_goal_trans,
                        y_ee,
                        0.01):
                    
                    self.box_name = 'lowerarm_r'
                    self.remove_box()
                    self.box_name = 'lowerarm_l'
                    self.remove_box()
                    state = 2
                    working = False
                    msg_oc=Bool()
                    msg_oc.data=False
                    pub_oc.publish(msg_oc)
                    
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
                self.human_collision()

                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_goal_trans
                goal_pose.position.y = y_goal_trans
                goal_pose.position.z = z_goal_trans
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                self.go_to_pose_goal(goal_pose)
            else:

                if self.difference(z_goal_trans, z_ee, 0.01):

                    self.box_name = 'lowerarm_r'
                    self.remove_box()
                    self.box_name = 'lowerarm_l'
                    self.remove_box()
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
                self.human_collision()

                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_ee
                goal_pose.position.y = y_ee
                goal_pose.position.z = 1
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                
                self.go_to_pose_goal(goal_pose)
            else:

                if self.difference(1, z_ee, 0.01):

                    self.box_name = 'lowerarm_r'
                    self.remove_box()
                    self.box_name = 'lowerarm_l'
                    self.remove_box()
                    state = 4
                    working = False
                    rospy.set_param("middleware",False)
        if state == 4:  # raggiungo la scatola

            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z
            if not working:
                print('stato 4')
                box = client_trans('Bluebox')
                x_box = box.transform.transform.translation.x
                y_box = box.transform.transform.translation.y
                z_box = box.transform.transform.translation.z
                print(x_box)
                print(y_box)
                print(z_box)
                self.human_collision()

                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_box
                goal_pose.position.y = y_box
                goal_pose.position.z = 1.2
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                self.go_to_pose_goal(goal_pose)
            else:

                if self.difference(
                        x_box,
                        x_ee,
                        0.01) & self.difference(
                        y_box,
                        y_ee,
                        0.01):

                    self.box_name = 'lowerarm_r'
                    self.remove_box()
                    self.box_name = 'lowerarm_l'
                    self.remove_box()
                    state = 5
                    working = False
        if state == 5:  # discesa
            x_ee = ee.position.x
            y_ee = ee.position.y
            z_ee = ee.position.z
            if not working:
                print('stato 5')
                self.human_collision()

                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x = x_ee
                goal_pose.position.y = y_ee
                goal_pose.position.z = z_box+0.2
                goal_pose.orientation.x = 0
                goal_pose.orientation.y = -1
                goal_pose.orientation.z = 0
                goal_pose.orientation.w = 0
                self.go_to_pose_goal(goal_pose)
            else:

                if self.difference(z_box+0.2, z_ee, 0.01):

                    self.box_name = 'lowerarm_r'
                    self.remove_box()
                    self.box_name = 'lowerarm_l'
                    self.remove_box()
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
    s1 = rospy.Service('/middleware', Bool, clbk)
    client_trans = rospy.ServiceProxy('/gl/transform', Transformation)
    sub_array = rospy.Subscriber("/blocks_state", BlocksState, clbk_array)
    sub = rospy.Subscriber("/left_gripper_pose", PoseStamped, clbk_ee)
    pub = rospy.Publisher('/baxter_moveit_trajectory',
                          BaxterTrajectory,
                          queue_size=20)
    pub_oc = rospy.Publisher('/open_close_left',
                          Bool,
                          queue_size=20)
                   
    g_left = GripperCommander()
    rospy.sleep(1)
    g_left.add_table()
    msg_oc=Bool()
    msg_oc.data=False
    pub_oc.publish(msg_oc)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        g_left.fsm()
        rate.sleep()
