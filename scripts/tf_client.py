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
from std_srvs.srv import *


if __name__ == "__main__":
        
        rospy.init_node("cl", anonymous=True)
        client_trans= rospy.ServiceProxy('/transform', Transformation)
        blue_box=client_trans('BlueBox')
        
        rospy.spin()
