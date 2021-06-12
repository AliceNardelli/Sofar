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
import numpy as np

def distance_between_blocks(f_block, s_block, threshold):
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
    rate = rospy.Rate(20)
    #blue_box_transformation = client_trans('Bluebox')

    while not rospy.is_shutdown():
        blue_box_transformation = client_trans('Bluebox')
        for i in range(5):

            # referred_block.frame_id = blocks_id[i]
            referred_block = client_trans(blocks_id[i])

            for j in range(11):
                if i != j:
                    other_block = client_trans(blocks_id[j])
                    # print(other_block)
                    # print(blocks_id[j])
                    # print(referred_block)
                    # print(blocks_id[i])
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
               # if i == 0:
                    #dis = math.sqrt(
                        #(referred_block.transform.transform.translation.x -
                         #blue_box_transformation.transform.transform.translation.x)**2 +
                        #(
                           # referred_block.transform.transform.translation.y -
                           # blue_box_transformation.transform.transform.translation.y)**2)
                    #print(dis)
                    # print(referred_block)
                    # print(blue_box_transformation)
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
