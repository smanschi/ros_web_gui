#!/usr/bin/env python3
import roslib
import rospy
import rostopic
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(f"I heard {data.data}")

topic_type = rostopic.get_topic_type('/chatter')
data_class = roslib.message.get_message_class(topic_type[0])

rospy.init_node('ros_test')
rospy.Subscriber("chatter", data_class, callback)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()