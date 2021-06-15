#!/usr/bin/env python  
"""
.. module:: tf_publisher
    :platform: Unix
    :synopsis: Python module for implementing the frame transformation

.. moduleauthor:: Giovanni Di Marco - imdimark@gmail.com
                  Alice Nardelli - alice.nardelli98@gmail.com
                  Federico Civetta - fedeunivers@gmail.com
                  Iacopo Pietrasanta - iacopo.pietrasanta@gmail.com

This node basically consists in an adapter which manage the frame transformation from Unity  to ROS domain 

Subscribes to:
     /unity_tf tf node in ROS
     

Publishes to:
    None

Service :
     None

Clients:
     None


"""
import rospy

# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from human_baxter_collaboration.msg import UnityTf

def publish_frames(msg):
    """
    Description of the publish_frame function:
           
    This function compute the transform within the 
    world frame
           
    
     Args :
             msg(TransformStamped) contains the trasnform
    
    Returns :
             None

    """
    br = tf2_ros.TransformBroadcaster()
    for transform in msg.frames:
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = transform.header.frame_id
        t.transform.translation.x = transform.pose.position.x
        t.transform.translation.y = transform.pose.position.y
        t.transform.translation.z = transform.pose.position.z

        t.transform.rotation.x = transform.pose.orientation.x
        t.transform.rotation.y = transform.pose.orientation.y
        t.transform.rotation.z = transform.pose.orientation.z
        t.transform.rotation.w = transform.pose.orientation.w

        br.sendTransform(t)

if __name__ == '__main__':
    """
    """
    rospy.init_node('unity_tf_publisher')
    rospy.Subscriber('unity_tf', UnityTf, publish_frames)
    rospy.spin()
