#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(f"I heard {data.data}")
    
rospy.init_node('subscriber')
rospy.Subscriber("chatter", String, callback)
# spin() simply keeps python from exiting until this node is stopped
rospy.spin()