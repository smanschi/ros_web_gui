import rosgraph
import roslib
import rosnode
import rosparam
import rospy
import rostopic
import socket
import sys
from datetime import datetime

class Topic():
    def __init__(self, name):
        super().__init__()
        self.__name = name
        topic_type_info = rostopic.get_topic_type(name)
        data_class = roslib.message.get_message_class(topic_type_info[0])
        self.__type = topic_type_info[0]
        self.__msg = None
        self.__msg_info = None
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
    def msg_stats(self):
        if self.__msg_info is None:
            return None
        return {
            'num_messages': self.__msg_info['num_messages'],
            'messages_per_second': float(self.__msg_info['num_messages']) / (datetime.now() - self.__msg_info['first_message']).total_seconds(),
            'last_message': self.__msg_info['last_message']
        }

    @property
    def subscribers(self):
        return self.__subs

    @property
    def publishers(self):
        return self.__pubs

    def clear(self):
        self.__pubs.clear()
        self.__subs.clear()

    def addPublisher(self, node):
        self.__pubs[node.name] = node

    def addSubscriber(self, node):
        self.__subs[node.name] = node

    def __onMessage(self, msg):
        if self.__msg_info is None:
            self.__msg_info = {
                'num_messages': 1,
                'first_message': datetime.now(),
                'last_message': None              
            }
        else:
            self.__msg_info['num_messages'] += 1
            self.__msg_info['last_message'] = datetime.now()

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
        self.__srvs[srv.name] = srv


class Service:
    def __init__(self, name):
        self.__name = name
        self.__type = rostopic.get_topic_type(name)[0]
        self.__providers = dict()

    @property
    def name(self):
        return self.__name

    @property
    def type(self):
        return self.__type

    @property
    def providers(self):
        return self.__providers

    def clear(self):
        self.__providers.clear()

    def addProvider(self, node):
        self.__providers[node.name] = node


class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class ROSApi(metaclass=Singleton):
    def __init__(self):
        print("Instance created")
        self.__nodes = dict()
        self.__topics = dict()
        self.__services = dict()
        self.__params = dict()
        self.__blacklisted_topics = set()
        self.__blacklisted_nodes = set()

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
    
    def config(self, blacklisted_topics, blacklisted_nodes):
        self.__blacklisted_topics = set([name if name[0] == '/' else '/' + name for name in blacklisted_topics])
        self.__blacklisted_nodes = set([name if name[0] == '/' else '/' + name for name in blacklisted_nodes])

    def get_topic(self, name):
        if name in self.__topics:
            return self.__topics[name]
        return None

    def get_node(self, name):
        if name in self.__nodes:
            return self.__nodes[name]
        return None

    def get_service(self, name):
        if name in self.__services:
            return self.__services[name]
        return None

    def update(self):
        # get master
        master = rosgraph.Master('rosnode')

        # go through the master system state first
        try:
            state = master.getSystemState()
        except socket.error:
            raise rosnode.ROSNodeIOException("Unable to communicate with master!")
        
        try:
            param_names = rosparam.list_params('')
        except rosparam.RosParamIOException:
            print('Could not fetch parameter names from server', file=sys.stdout)
        self.__params = sorted(set(['/'.join(param.split('/')[:2]) for param in param_names]))

        # Clear state
        # Topics will not be removed to avoid killing the topic subscribers
        self.__nodes.clear()
        self.__services.clear()

        # Iterate over publisher topics and create nodes and topics if necessary
        for s in state[0]:
            topic_name = s[0]
            if topic_name not in self.__topics and topic_name not in self.__blacklisted_topics:
                self.__topics[topic_name] = Topic(topic_name)
            for node_name in s[1]:
                if node_name not in self.__blacklisted_nodes:
                    if node_name not in self.__nodes:
                        self.__nodes[node_name] = Node(node_name)
                    if topic_name not in self.__blacklisted_topics:
                        self.__topics[topic_name].addPublisher(self.__nodes[node_name])
                        self.__nodes[node_name].addPublication(self.__topics[topic_name])

        # Iterate over subscriber topics and create nodes and topics if necessary
        for s in state[1]:
            topic_name = s[0]
            if topic_name not in self.__topics and topic_name not in self.__blacklisted_topics:
                self.__topics[topic_name] = Topic(topic_name)
            for node_name in s[1]:
                if node_name not in self.__blacklisted_nodes:
                    if node_name not in self.__nodes:
                        self.__nodes[node_name] = Node(node_name)
                    if topic_name not in self.__blacklisted_topics:
                        self.__topics[topic_name].addSubscriber(self.__nodes[node_name])
                        self.__nodes[node_name].addSubscription(self.__topics[topic_name])

        # Delete blacklisted topics
        remove = [topic_name for topic_name in self.__topics if topic_name in self.__blacklisted_topics]
        for topic_name in remove: del self.__topics[topic_name]

        # Iterate over service topics and create nodes and services if necessary
        for s in state[2]:
            service_name = s[0]
            if service_name not in self.__services:
                self.__services[service_name] = Service(service_name)
            self.__services[service_name].clear()
            for node_name in s[1]:
                if node_name not in self.__blacklisted_nodes:
                    if node_name not in self.__nodes:
                        self.__nodes[node_name] = Node(node_name)
                    self.__services[service_name].addProvider(self.__nodes[node_name])
                    self.__nodes[node_name].addService(self.__services[service_name])
            # Remove service if it does not have a provider
            if len(self.__services[service_name].providers) == 0:
                del self.__services[service_name]

ros = ROSApi()
#rospy.init_node('ros_web_gui')
#_thread.start_new_thread(rospy.spin, ()) 
