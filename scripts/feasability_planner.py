#!/usr/bin/env python

"""
.. module:: feasability_planner
    :platform: Unix
    :synopsis: Python module for implementing the pheasibility planner

.. moduleauthor:: Giovanni Di Marco - imdimark@gmail.com
                  Alice Nardelli - alice.nardelli98@gmail.com
                  Federico Civetta - fedeunivers@gmail.com
                  Iacopo Pietrasanta - iacopo.pietrasanta@gmail.com

This  component  is  involved  in  the  perception  and  scene  analysis.
It  continuously  asks  totf_listener.pyfor  the  transformation  of  each  
blue  block  on  the  table.   Then  it  publishes an array with 
how blocks are arranged in the scene

Subscribes to:
     None

Publishes to:
    /blocks_state

Service :
     None 

Clients:
     /gl/transform


"""
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from human_baxter_collaboration.msg import UnityTf
from tf import transformations
from std_srvs.srv import *
from human_baxter_collaboration.srv import Transformation
from human_baxter_collaboration.msg import BlocksState
import math
import numpy as np

def distance_between_blocks(f_block, s_block, threshold):
    """
    Description of the distance_between_blocks function:
           
    the planar distance between the two transform is comuputed.
    if the distance is greater than the threshold is return false
    if the distance is below z value is compared
    if z of the first transform is below the z of the second transform is return true
    otherwise is return false
           
    
    Args :
      f_block(Transformation) a tranform of a block
      s_block(Transformation) a tranform of a block
      threshold(float64)
    
    Returns :
      bool
      
    """
    dis = math.sqrt((f_block.transform.transform.translation.x -
                     s_block.transform.transform.translation.x)**2 +
                    (f_block.transform.transform.translation.y -
                     s_block.transform.transform.translation.y)**2)

    if dis < threshold:

        if f_block.transform.transform.translation.z <= s_block.transform.transform.translation.z:
            return True
        else:
            return False
    else:
        return False


if __name__ == '__main__':
    """
    Description of the main function:
           
    inside this function the feasability planner is computed
           
    
    Args :
      None
    
    Returns :
      None

    """
    rospy.sleep(3)
    global blocks_state, blocks_id
    global client_trans, pub

    rospy.init_node('feasability_planner')
    blocks_state = [4, 4, 4, 4, 4]
    blocks_id = ['C', 'E', 'G', 'I', 'M', 'A', 'B', 'D', 'F', 'H', 'L']
    #client of /gl/transform to retrieve transforms
    client_trans = rospy.ServiceProxy('/gl/transform', Transformation)
    #a publisher of /blocks_state is used to plublish the state of each blocks
    pub = rospy.Publisher('/blocks_state', BlocksState, queue_size=1)
    state_message = BlocksState()
    crowded = False
    rate = rospy.Rate(15)
    

    while not rospy.is_shutdown():
        blue_box_transformation = client_trans('Bluebox')
        #for each blue box is computed the state
        for i in range(5):

           
            referred_block = client_trans(blocks_id[i])
            #for this block is computed the mutual position with all others blue and red boxes
            for j in range(11):
                if i != j:
                    #get the transform of the other block and check if it is above the block under analysis 
                    other_block = client_trans(blocks_id[j])
                    crowded = distance_between_blocks(
                            referred_block, other_block, 0.05)

                #crowded is true if a block is above another so it is considered as non feasible
                if crowded:
                    #if it is above the blue box its state is zero: inside the blue box. otherwise is four
                    if distance_between_blocks(
                         blue_box_transformation,referred_block, 0.2):
                       blocks_state[i] = 0
                    else:
                       blocks_state[i] = 4
                    break
            #if it is feasible
            if not crowded:
                #if it is above the blue box its state is zero
                if distance_between_blocks(
                         blue_box_transformation,referred_block, 0.2):
                    blocks_state[i] = 0
                else:
                    #if it is at the centre state is 2
                    if np.abs(-referred_block.transform.transform.translation.y) < 0.1:
                        blocks_state[i] = 2
                    #if it is at right state is 1    
                    elif -referred_block.transform.transform.translation.y > 0:
                        blocks_state[i] = 1
                    #if it is at left state is 3    
                    elif -referred_block.transform.transform.translation.y < 0:
                        blocks_state[i] = 3
            #le blocks_array is published
            state_message.blocksarray = blocks_state
            pub.publish(state_message)

        rate.sleep()
