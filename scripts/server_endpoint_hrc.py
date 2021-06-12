#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from human_baxter_collaboration.msg import BaxterTrajectory, UnityTf
from geometry_msgs.msg import Quaternion, Pose,  PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)
    rospy.init_node(ros_node_name, anonymous=True)

    # Start the Server Endpoint with a ROS communication objects dictionary for routing messages
    tcp_server.start({
    	'unity_tf': RosPublisher('unity_tf', UnityTf, queue_size=100),
    	'baxter_joint_states': RosPublisher('baxter_joint_states', JointState, queue_size=100),
    	'left_gripper_pose': RosPublisher('left_gripper_pose', PoseStamped, queue_size=100),
    	'right_gripper_pose': RosPublisher('right_gripper_pose', PoseStamped, queue_size=100),
    	'baxter_moveit_trajectory': RosSubscriber('baxter_moveit_trajectory', BaxterTrajectory, tcp_server),
    	'open_close_right': RosSubscriber('open_close_right', Bool, tcp_server),
    	'open_close_left': RosSubscriber('open_close_left', Bool, tcp_server),
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()
