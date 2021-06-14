#!/usr/bin/env python
"""
.. module:: tf_listener
    :platform: Unix
    :synopsis: Python module for implementing the transfromation frame tf_listener

.. moduleauthor:: Giovanni Di Marco - imdimark@gmail.com
                  Alice Nardelli - alice.nardelli98@gmail.com
                  Federico Civetta - fedeunivers@gmail.com
                  Iacopo Pietrasanta - iacopo.pietrasanta@gmail.com


This is a service node which is charachterized as follows:
1. as request:  it listens the object transformations  from tf_publisher.py and 
2. as response: it sends the listened object tranformations to another nodes.


Subscribes to:
     /blocks_state
     /left_gripper_pose

Publishes to:
    None
    None

Service :
     /gl/transform 

Clients:
     None
     


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
# listener=None
# tfBuffer=None
# rate=0


def clbk(req):
    """
    Description of the '' function:
           
    lore ipsum
           
    
     Args :
             None
    
    Returns :
             None

    """
    global listener
    global tfBuffer
    global rate
    t = geometry_msgs.msg.TransformStamped()
    try:
        t = tfBuffer.lookup_transform('world', req.frame_id, rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        
    return t


if __name__ == '__main__':
    """
    """
    global listener
    global tfBuffer
    global rate
    rospy.init_node('tf_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    s1 = rospy.Service('/gl/transform', Transformation, clbk)
    s2 = rospy.Service('/gr/transform', Transformation, clbk)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()
