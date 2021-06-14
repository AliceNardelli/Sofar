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
    Description of the '' function:
           
    lore ipsum
           
    
    Args :
      None
    
    Returns :
      None
      
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
    Description of the '' function:
           
    lore ipsum
           
    
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
    client_trans = rospy.ServiceProxy('/gl/transform', Transformation)
    pub = rospy.Publisher('/blocks_state', BlocksState, queue_size=1)
    state_message = BlocksState()
    crowded = False
    rate = rospy.Rate(15)
    

    while not rospy.is_shutdown():
        blue_box_transformation = client_trans('Bluebox')
        for i in range(5):

           
            referred_block = client_trans(blocks_id[i])

            for j in range(11):
                if i != j:
                    other_block = client_trans(blocks_id[j])

                    if j < 5:
                        crowded = distance_between_blocks(
                            referred_block, other_block, 0.05)
                    else:
                        crowded = distance_between_blocks(
                            referred_block, other_block, 0.05)

                if crowded:

                    blocks_state[i] = 4
                    break

            if not crowded:

                if distance_between_blocks(
                         blue_box_transformation,referred_block, 0.2):
                    blocks_state[i] = 0
                else:
                    if np.abs(-referred_block.transform.transform.translation.y) < 0.1:
                        blocks_state[i] = 2
                        
                    elif -referred_block.transform.transform.translation.y > 0:
                        blocks_state[i] = 1

                    elif -referred_block.transform.transform.translation.y < 0:
                        blocks_state[i] = 3

            state_message.blocksarray = blocks_state
            pub.publish(state_message)

        rate.sleep()
