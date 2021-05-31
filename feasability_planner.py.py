#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from human_baxter_collaboration.msg import UnityTf
from tf import transformations
from std_srvs.srv import *

blocks_state=[4, 4, 4, 4, 4]
blocks_id=['C', 'E', 'G','I', 'M', 'A', 'B', 'D', 'F', 'H', 'L']
client_trans=None
pub=None

def distance_between_blocks(f_block, s_block, threshold):
    dis=sqrt((f_block.transform.transform.translation.x - s_block.transform.transform.translation.x )**2 + (f_block.transform.transform.translation.y - s_block.transform.transform.translation.y )**2)
    if dis < threshold:
        if f_block.transform.transform.translation.z <= s_block.transform.transform.translation.z:
            return True
        else: 
            return False
    else:
        False

if __name__ == '__main__':

    global blocks_state, blocks_id
    global client_trans, pub

    rospy.init_node('feasability_planner')
    client_trans= rospy.ServiceProxy('transform', Transformation)
    pub = rospy.Publisher('blocks_state', BlocksState , queue_size=1)
    state_message = BlocksState()
    blue_box_transformation = Trasformation()
    referred_block=Transformation()
    other_block=Transformation()
    crowded=False
    rate= rospy.Rate(20)
    blue_box_transformation.frame_id= 'BlueBox'
    blue_box_transformation.transform=client_trans(blue_box_transformation.frame_id)

    while not rospy.is_shutdown():
        
        for i in range(0:5)

            referred_block.frame_id=blocks_id[i]
            referred_block.transform=client_trans(referred_block.frame_id)

            for j in range (0:11):
                if i!=j:
                    other_block.transform=client_trans(blocks_id[j])
                    crowded=distance_between_blocks(referred_block.transform, other_block.transform, 2.5)
                if crowded==True:
                    blocks_state[i]=4
                    break

            if referred_block.transform.transform.translation.x>0:
                blocks_state[i]=1
                
            if referred_block.transform.transform.translation.x==0:
                blocks_state[i]=2
            ##setto parametro middleplacent true RICORDA

            if referred_block.transform.transform.translation.x<0:
                blocks_state[i]=3

            if  (referred_block.transform, blue_box.transformation.transform, 10) = True: 
                blocks_state[i]=0
        
        state_message.blocksarray= blocks_state
        pub.publish(state_message)

        rate.sleep()
    