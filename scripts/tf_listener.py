#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from human_baxter_collaboration.msg import UnityTf
from tf import transformations
from std_srvs.srv import *

global listener
global tfBuffer


def clbk(req):
      global listener
      global tfBuffer
      try:
	    trans= tfBuffer.lookup_transform(req.frame_id,'world', rospy.Time(0))

      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
	    rate.sleep()
	    continue
      return TransformationResponse(trans)
	


if __name__ == '__main__':
        global listener
        global tfBuffer
	rospy.init_node('tf_listener')   
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	s = rospy.Service('transform', Transformation, clbk)
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
                   rate.sleep()
		
		
	
