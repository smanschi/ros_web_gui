#!/usr/bin/env python3
import roslib
import rospy
import rostopic
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(f"Got message: {data}")

topic_name = '/chatter'
topic_type = rostopic.get_topic_type(topic_name)
data_class = roslib.message.get_message_class(topic_type[0])

rospy.init_node('ros_test')
rospy.Subscriber(topic_name, data_class, callback)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
