import rosgraph
import roslib
import rosnode
import rosparam
import rospy
import rostopic
import socket
import sys

def get_info():
    # get master
    master = rosgraph.Master('rosnode')

    # go through the master system state first
    try:
        state = master.getSystemState()
    except socket.error:
        raise rosnode.ROSNodeIOException("Unable to communicate with master!")
    
    param_names = []
    try:
        param_names = rosparam.list_params('')
    except rosparam.RosParamIOException:
        print('Could not fetch parameter names from server', file=sys.stdout)
    param_names = sorted(set(['/'.join(param.split('/')[:2]) for param in param_names]))

    return {
        'pubs':   [{'topic': s[0], 'publisher':  s[1], 'type': rostopic.get_topic_type(s[0])[0]} for s in state[0]],
        'subs':   [{'topic': s[0], 'subscriber': s[1], 'type': rostopic.get_topic_type(s[0])[0]} for s in state[1]],
        'srvs':   [{'topic': s[0], 'provider':   s[1], 'type': rostopic.get_topic_type(s[0])[0]} for s in state[2]],
        'params': [{'name': s, 'type': 'unknown'} for s in param_names]
    }


class Topic:
    def __init__(self, name):
        self.__name = name
        topic_type_info = rostopic.get_topic_type(name)
        data_class = roslib.message.get_message_class(topic_type_info[0])
        self.__type = topic_type_info[0]
        self.__msg = None
        self.__ros_subscriber = rospy.Subscriber(name, data_class, self.__onMessage)
        self.__subs = dict()
        self.__pubs = dict()

    @property
    def name(self):
        return self.__name

    @property
    def type(self):
        return self.__type

    @property
    def msg(self):
        return self.__msg

    @property
    def subscribers(self):
        return self.__subs

    @property
    def publishers(self):
        return self.__pubs

    def addPublisher(self, node):
        self.__pubs[node.name] = node

    def addSubscriber(self, node):
        self.__subs[node.name] = node

    def __onMessage(self, msg):
        self.__msg = msg


class Node:
    def __init__(self, name):
        self.__name = name
        self.__subs = dict()
        self.__pubs = dict()
        self.__srvs = dict()

    @property
    def name(self):
        return self.__name

    @property
    def subscriptions(self):
        return self.__subs

    @property
    def publications(self):
        return self.__pubs

    @property
    def services(self):
        return self.__srvs

    def addSubscription(self, sub):
        self.__subs[sub.name] = sub

    def addPublication(self, pub):
        self.__pubs[pub.name] = pub

    def addService(self, srv):
        self.__pubs[srv.name] = srv


class Service:
    def __init__(self, name):
        self.__name = name
        self.__type = rostopic.get_topic_type(name)[0]
        self.__provider = dict()

    @property
    def name(self):
        return self.__name

    @property
    def type(self):
        return self.__type

    @property
    def providers(self):
        return self.__provider

    def addProvider(self, node):
        self.__provider[node.name] = node


class ROSApi:
    def __init__(self):
        print("Instance created")
        self.__nodes = dict()
        self.__topics = dict()
        self.__services = dict()
        self.__params = dict()

    @property
    def topics(self):
        return self.__topics
    
    @property
    def nodes(self):
        return self.__nodes

    @property
    def services(self):
        return self.__services

    @property
    def params(self):
        return self.__params

    def update(self):
        # get master
        master = rosgraph.Master('rosnode')

        # go through the master system state first
        try:
            state = master.getSystemState()
        except socket.error:
            raise rosnode.ROSNodeIOException("Unable to communicate with master!")
        
        param_names = []
        try:
            param_names = rosparam.list_params('')
        except rosparam.RosParamIOException:
            print('Could not fetch parameter names from server', file=sys.stdout)
        param_names = sorted(set(['/'.join(param.split('/')[:2]) for param in param_names]))

        # Iterate over publisher topics and create nodes and topics if necessary
        for s in state[0]:
            topic_name = s[0]
            if topic_name not in self.__topics:
                self.__topics[topic_name] = Topic(topic_name)
            for node_name in s[1]:
                if node_name not in self.__nodes:
                    self.__nodes[node_name] = Node(node_name)
                self.__topics[topic_name].addPublisher(self.__nodes[node_name])
                self.__nodes[node_name].addPublication(self.__topics[topic_name])

        # Iterate over subscriber topics and create nodes and topics if necessary
        for s in state[1]:
            topic_name = s[0]
            if topic_name not in self.__topics:
                self.__topics[topic_name] = Topic(topic_name)
            for node_name in s[1]:
                if node_name not in self.__nodes:
                    self.__nodes[node_name] = Node(node_name)
                self.__topics[topic_name].addSubscriber(self.__nodes[node_name])
                self.__nodes[node_name].addSubscription(self.__topics[topic_name])

        # Iterate over service topics and create nodes and services if necessary
        for s in state[2]:
            service_name = s[0]
            if service_name not in self.__services:
                self.__services[service_name] = Service(service_name)
            for node_name in s[1]:
                if node_name not in self.__nodes:
                    self.__nodes[node_name] = Node(node_name)
                self.__services[topic_name].addProvider(self.__nodes[node_name])
                self.__nodes[node_name].addService(self.__services[topic_name])

        return {
            'pubs':   [{'topic': s[0], 'publisher':  s[1], 'type': rostopic.get_topic_type(s[0])[0]} for s in state[0]],
            'subs':   [{'topic': s[0], 'subscriber': s[1], 'type': rostopic.get_topic_type(s[0])[0]} for s in state[1]],
            'srvs':   [{'topic': s[0], 'provider':   s[1], 'type': rostopic.get_topic_type(s[0])[0]} for s in state[2]],
            'params': [{'name': s, 'type': 'unknown'} for s in param_names]
        }

api = ROSApi()