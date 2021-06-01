#!/usr/bin/env python
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


def distance_between_blocks(f_block, s_block, threshold):
    dis = math.sqrt((f_block.transform.transform.translation.y -
                     s_block.transform.transform.translation.y)**2 +
                    (f_block.transform.transform.translation.x -
                     s_block.transform.transform.translation.x)**2)
                  
    if dis < threshold:

        if f_block.transform.transform.translation.z <= s_block.transform.transform.translation.z:
            return True
        else:
            return False
    else:
        return False

def distance_between_blocks_blue(f_block, s_block, threshold):
    dis = math.sqrt((f_block.transform.transform.translation.y -
                     s_block.transform.transform.translation.y)**2 +
                    (f_block.transform.transform.translation.z -
                     s_block.transform.transform.translation.z)**2)
    print(dis)                 
    if dis < threshold:

        if f_block.transform.transform.translation.x <= s_block.transform.transform.translation.x:
            return True
        else:
            return False
    else:
        return False


def distance_between_blocks_red(f_block, s_block, threshold):
    dis = math.sqrt((f_block.transform.transform.translation.y +
                     s_block.transform.transform.translation.x)**2 +
                    (f_block.transform.transform.translation.z -
                     s_block.transform.transform.translation.z)**2)
    
    if dis < threshold:

        if f_block.transform.transform.translation.x <= s_block.transform.transform.translation.y:
            return True
        else:
            return False
    else:
        return False


if __name__ == '__main__':

    global blocks_state, blocks_id
    global client_trans, pub

    rospy.init_node('feasability_planner')
    blocks_state = [4, 4, 4, 4, 4]
    blocks_id = ['C', 'E', 'G', 'I', 'M', 'A', 'B', 'D', 'F', 'H', 'L']
    client_trans = rospy.ServiceProxy('/transform', Transformation)
    pub = rospy.Publisher('/blocks_state', BlocksState, queue_size=1)
    state_message = BlocksState()
    crowded = False
    rate = rospy.Rate(20)
    blue_box_transformation = client_trans('BlueBox')

    while not rospy.is_shutdown():

        for i in range(5):

            # referred_block.frame_id = blocks_id[i]
            referred_block = client_trans(blocks_id[i])

            for j in range(11):
                if i != j:
                    other_block = client_trans(blocks_id[j])
                    if j < 5:
                        crowded = distance_between_blocks(
                            referred_block, other_block, 0.1)
                    else:
                        crowded = distance_between_blocks(
                            referred_block, other_block, 0.1)
                    
                if crowded:
                    
                    blocks_state[i] = 4
                    break
                print('\n')
            if not crowded:
                if referred_block.transform.transform.translation.y > 0:
                    blocks_state[i] = 1

                if referred_block.transform.transform.translation.y == 0:
                    blocks_state[i] = 2
                # setto parametro middleplacent true RICORDA

                if referred_block.transform.transform.translation.y < 0:
                    blocks_state[i] = 3

                if distance_between_blocks(
                        referred_block, blue_box_transformation, 0.1):
                    blocks_state[i] = 0

            state_message.blocksarray = blocks_state
            pub.publish(state_message)

        rate.sleep()
