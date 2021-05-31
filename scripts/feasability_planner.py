
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

if __name__ == '__main__':
        global blocks_state, blocks_id
        global client_trans, pub
	rospy.init_node('feasability_planner')
	client_trans= rospy.ServiceProxy('transform', Transformation)   
        pub = rospy.Publisher('blocks_state', BlocksState , queue_size=1)
        message=Transformation()        
        crowded=False
	rate= rospy.Rate(20)
	
	while not rospy.is_shutdown():
	           for i in range(0:5)
	                message.frame_id=blocks_id[i]
	                message.transform=client_trans(message.frame_id)
	                for j in range (0:11):
	                    if i!=j:
	                         crowded=distance_between_blocks(blocks_id[i],blocks_id[j])
	                         if crowded==True:
	                             blocks_state[i]=4;
	                             break;
	                     
			if message.transform.transform.translation.x>0:
				blocks_state[i]=1
				             
			if message.transform.transform.translation.x==0:
			        blocks_state[i]=2
				             
				##setto parametro middleplacent true RICORDA
			if message.transform.transform.translation.x<0:
				blocks_state[i]=3
				             
			se (x,y)==bluebox
				blocks_state[i]=3 
                   rate.sleep()
