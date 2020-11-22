#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

def callback(data):
    for idx, name in enumerate(data.name):
        print(f"idx: {idx} name: {name} position: {data.position[idx]}")
    
rospy.init_node('subscriber_joints')
rospy.Subscriber("/joint_states", JointState, callback)
# spin() simply keeps python from exiting until this node is stopped
rospy.spin()