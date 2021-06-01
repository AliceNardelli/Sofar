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
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle

state=0
end=False
working=False
blocks_id=['C', 'E', 'G','I', 'M']

def clbk_array(msg):
    global array_blocks
    for i in range(0,5)
      array_blocks[i]=msg.blocksarray[i]
    


          
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
        self.box_name = ""
     


    def go_to_pose_goal(self, pose_goal):              
        (self.move_group).setStartState(*((self.move_group).getCurrentState()))
        (self.move_group).setGoalTolerance(0.001)
        (self.move_group).set_pose_target(pose_goal)
        #plan+execute
        
        plan = self.move_group.plan()
        self.trajectory_pub.publish(plan)
        rospy.loginfo (plan)
        self.move_group.stop()
        
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.001)
        
    def add_table(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = 'Table'
        box_pose.pose.position.x=0
        box_pose.pose.position.y=0.4
        box_pose.pose.position.z =0.7673182  # above the panda_hand frame
        box_name = "Table"
        scene.add_box(box_name, box_pose, size=(2, 0.8, 0.6))
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
        
    def add_cylinder(self, timeout=4,f_id,pose_obs):
        box_name = self.box_name
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = f_id
        box_pose.pose.position.x=pose_obs.transform.transform.translation.x
        box_pose.pose.position.y=pose_obs.transform.transform.translation.y
        box_pose.pose.position.z=pose_obs.transform.transform.translation.z
        box_pose.pose.orientation.x=pose_obs.transform.transform.rotation.x
        box_pose.pose.orientation.y=pose_obs.transform.transform.rotation.y
        box_pose.pose.orientation.z=pose_obs.transform.transform.rotation.z
        box_pose.pose.orientation.w=pose_obs.transform.transform.rotation.w
        box_name = f_id
        scene.add_cylinder(box_name, box_pose,25,0.05)
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
        
    def remove_box(self, timeout=4):
	 box_name = self.box_name
	 scene = self.scene
	 scene.remove_world_object(box_name)
	 return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
    
    def human_collision(self):
        global working,client_trans
        trans_right=client_trans('upperarm_r')
        trans_left=client_trans('upperarm_l')
        self.add_cylinder('upperarm_r',trans_right)
        self.add_cylinder('upperarm_l',trans_left)
        working=True
          
    def fsm(self):
        global state, blocks_array,end,working,blocks_id
        selected_position =0
        if state==0:  #rest
            if blocks_array[0]==0 && blocks_array[1]==0 && blocks_array[2]==0 && blocks_array[3]==0 && blocks_array[4]==0:
                  end=True
            else:
                   for i in range(0,5):
                         if blocks_array[i]==2 || blocks_array[i]==3:
                               state=2 
        if state==1:  #reaching
                   middleware=rospy.get_param('middleware')
                   
                   if middleware==True:
                     for j in range (0,5):
                     	if blocks_array[j] == 2:
                     		selected_position = j
                     		break
                   else:  #mettiamo i blocchi in modo tale che i primi a essere presi sono quelli a destra?
                      for j in range (0,5):
                     	if blocks_array[j] == 3:
                     		selected_position = j
                     		break
                   goal_trans=client_trans(blocks_id[selected_position])
                   x_goal_trans=goal_trans.transform.transform.translation.x
                   y_goal_trans=goal_trans.transform.transform.translation.y
                   x_ee=self.move_group.get_current_pose().pose.position.x
                   y_ee=self.move_group.get_current_pose().pose.position.y
                   z_ee=self.move_group.get_current_pose().pose.position.z
                   if working==False:
                         
                         self.human_collision()
                         
                         goal_pose = geometry_msgs.msg.Pose() 
                         goal_pose.position.x = x_goal_trans
                         goal_pose.position.y =y_goal_trans
                         goal_pose.position.z = z_ee
                         self.go_to_pose_goal(goal_pose)
                   else: 
                         
                         if (x_goal_trans,x_ee,0.05)&&(y_goal_trans,y_ee,0.05):
                            
                             self.box_name='upperarm_r'
                             self.remove_box()
                             self.box_name='upperarm_l'  
                             self.remove_box()
                             state=2
        if state==2:  #discesa

                   x_ee=self.move_group.get_current_pose().pose.position.x
                   y_ee=self.move_group.get_current_pose().pose.position.y
                   z_ee=self.move_group.get_current_pose().pose.position.z
                   if working==False:
                         
                         self.human_collision()
                         
                         goal_pose = geometry_msgs.msg.Pose() 
                         goal_pose.position.x = x_ee
                         goal_pose.position.y =y_ee
                         goal_pose.position.z = 0
                         self.go_to_pose_goal(goal_pose)
                   else: 
                         
                         if (0,z_ee,0.05):
                            
                             self.box_name='upperarm_r'
                             self.remove_box()
                             self.box_name='upperarm_l'  
                             self.remove_box()
                             state=3
        if state==3:  #sollevamento
                   x_ee=self.move_group.get_current_pose().pose.position.x
                   y_ee=self.move_group.get_current_pose().pose.position.y
                   z_ee=self.move_group.get_current_pose().pose.position.z
                   if working==False:
                         
                         self.human_collision()
                         
                         goal_pose = geometry_msgs.msg.Pose() 
                         goal_pose.position.x = x_ee
                         goal_pose.position.y =y_ee
                         goal_pose.position.z = 0.5
                         self.go_to_pose_goal(goal_pose)
                   else: 
                         
                         if (0.5,z_ee,0.05):
                            
                             self.box_name='upperarm_r'
                             self.remove_box()
                             self.box_name='upperarm_l'  
                             self.remove_box()
                             state=4
        if state==4:  #raggiungo la scatola
                   goal_trans=client_trans('BlueBox')
                   x_goal_trans=goal_trans.transform.transform.translation.x
                   y_goal_trans=goal_trans.transform.transform.translation.y
                   x_ee=self.move_group.get_current_pose().pose.position.x
                   y_ee=self.move_group.get_current_pose().pose.position.y
                   z_ee=self.move_group.get_current_pose().pose.position.z
                   if working==False:
                         
                         self.human_collision()
                         
                         goal_pose = geometry_msgs.msg.Pose() 
                         goal_pose.position.x = x_goal_trans
                         goal_pose.position.y =y_goal_trans
                         goal_pose.position.z = z_ee
                         self.go_to_pose_goal(goal_pose)
                   else: 
                         
                         if (x_goal_trans,x_ee,0.1)&&(y_goal_trans,y_ee,0.1):
                            
                             self.box_name='upperarm_r'
                             self.remove_box()
                             self.box_name='upperarm_l'  
                             self.remove_box()
                             state=5
        if state==5:  #discesa
                  
                   x_ee=self.move_group.get_current_pose().pose.position.x
                   y_ee=self.move_group.get_current_pose().pose.position.y
                   z_ee=self.move_group.get_current_pose().pose.position.z
                   if working==False:
                         
                         self.human_collision()
                         
                         goal_pose = geometry_msgs.msg.Pose() 
                         goal_pose.position.x = x_ee
                         goal_pose.position.y =y_ee
                         goal_pose.position.z = 0
                         self.go_to_pose_goal(goal_pose)
                   else: 
                         
                         if (0,z_ee,0.05):
                            
                             self.box_name='upperarm_r'
                             self.remove_box()
                             self.box_name='upperarm_l'  
                             self.remove_box()
                             state=0
       
             
if __name__ == "__main__":
        global blocks_array,client_trans, state,working,end
        global client_trans
        rospy.init_node("fsm_right")
        client_trans= rospy.ServiceProxy('transform', Transformation)
        sub_array=rospy.Subscriber("/blocks_state", BlocksState, clbk_array)
        g_right = GripperCommander()
        g_right.add_table()
        
        rate= rospy.Rate(20)
	
	while not rospy.is_shutdown():
	    g_right.fsm()
	    rate.sleep()

