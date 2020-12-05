import genpy
import pygraphviz as pgv
import roscpp
import rosgraph
import roslib
import rosnode
import rosparam
import rospy
import rosservice
import rostopic
import socket
import sys
from datetime import datetime
from flask import Markup, url_for
from io import BytesIO
from .util import get_object_size, str_to_msg, initialize_msg

class Topic():
    def __init__(self, name):
        super().__init__()
        rospy.loginfo(f"Instantiating topic with name {name}")
        self.__name = name
        self.__msg = None
        self.__msg_info = None
        try:
            topic_type_info = rostopic.get_topic_type(name)
            self.__type = topic_type_info[0]
        except Exception as e:
            rospy.logerr(str(e))
            self.type = None
        try:
            self.__data_class = roslib.message.get_message_class(topic_type_info[0])
            self.__msg_template = initialize_msg(self.__data_class())
            self.__ros_subscriber = rospy.Subscriber(name, self.__data_class, self.__onMessage)
            self.__ros_publisher = rospy.Publisher(name, self.__data_class, queue_size=1)
        except Exception as e:
            rospy.logerr(str(e))
            self.__data_class = None
            self.__ros_subscriber = None
            self.__ros_publisher = None
        self.__subs = dict()
        self.__pubs = dict()
        self.__graph = None
        self.__svg = None
        self.__times = {
            'state': datetime.now(),
            'graph': None,
            'svg': None
        }

    @property
    def name(self):
        return self.__name

    @property
    def type(self):
        return self.__type

    @property
    def msg(self):
        if self.__msg_info is None:
            return None
        return {
            'msg': self.__msg,
            'num_messages': self.__msg_info['num_messages'],
            'messages_per_second': float(self.__msg_info['num_messages']) / (datetime.now() - self.__msg_info['first_message']).total_seconds(),
            'last_message': self.__msg_info['last_message'],
            'avg_size': self.__msg_info['avg_size']
        }

    @property
    def data_class(self):
        return self.__data_class

    @property
    def subscribers(self):
        return self.__subs

    @property
    def publishers(self):
        return self.__pubs

    def clear(self):
        self.__pubs.clear()
        self.__subs.clear()
        self.__times['state'] = datetime.now()

    def unregister(self):
        if self.__ros_subscriber is not None:
            self.__ros_subscriber.unregister()
        if self.__ros_publisher is not None:
            self.__ros_publisher.unregister()

    def addPublisher(self, node):
        self.__pubs[node.name] = node
        self.__times['state'] = datetime.now()

    def addSubscriber(self, node):
        self.__subs[node.name] = node
        self.__times['state'] = datetime.now()

    def publish(self, msg_str):
        try:
            msg = str_to_msg(msg_str, self.__ros_publisher.data_class)
            self.__ros_publisher.publish(msg)
        except Exception as e:
            rospy.loginfo(e)
            raise e

    def graph(self):
        # Check if we can return an existing graph
        if (self.__graph is not None) and (self.__times['graph'] >= self.__times['state']):
            return self.__graph

        # Remember time
        rospy.loginfo(f'Generating graph for topic {self.__name}')
        self.__times['graph'] = datetime.now()

        # Generate new graph
        graph = pgv.AGraph(directed=True, forcelabels=True)
        topic_id = 'topic_' + self.__name
        topic_label = f"{self.__name}\\n{self.__type}"
        graph.add_node(topic_id, label=topic_label, shape='box')

        # Add publishers
        for node_name in self.__pubs:
            node_id = 'pub_' + node_name
            node_url = url_for('node.get_node_info', name=node_name)
            graph.add_node(node_id, label=node_name, shape='oval', URL=node_url, target='_top')
            graph.add_edge(node_id, topic_id)

        # Add subscribers
        for node_name in self.__subs:
            node_id = 'sub_' + node_name
            node_url = url_for('node.get_node_info', name=node_name)
            graph.add_node(node_id, label=node_name, shape='oval', URL=node_url, target='_top')
            graph.add_edge(topic_id, node_id)

        # Store and return graph
        self.__graph = graph
        return graph

    def svg(self):
        # Check if we can return an existing svg
        graph = self.graph()
        if (self.__svg is not None) and (self.__times['svg'] >= self.__times['graph']):
            return self.__svg

        # Remember time
        rospy.loginfo(f'Generating svg for topic {self.__name}')
        self.__times['svg'] = datetime.now()

        # Generate svg
        img_stream = BytesIO()
        graph.draw(path=img_stream, format='svg', prog='dot')
        svg = img_stream.getvalue().decode('utf-8')
        svg = svg.replace('xlink:', '')

        # Store and return svg
        self.__svg = svg
        return svg

    def msg_template(self):
        return genpy.message.strify_message(self.__msg_template)

    def __onMessage(self, msg):
        if self.__msg_info is None:
            self.__msg_info = {
                'num_messages': 1,
                'first_message': datetime.now(),
                'last_message': datetime.now(),
                'avg_size': get_object_size(msg)          
            }
        else:
            self.__msg_info['num_messages'] += 1
            self.__msg_info['last_message'] = datetime.now()
            self.__msg_info['avg_size'] = (self.__msg_info['avg_size']*(self.__msg_info['num_messages']-1) + get_object_size(msg))/self.__msg_info['num_messages']

        self.__msg = msg


class Node:
    def __init__(self, name):
        self.__name = name
        self.__subs = dict()
        self.__pubs = dict()
        self.__srvs = dict()
        self.__graph = None
        self.__svg = None
        self.__times = {
            'state': datetime.now(),
            'graph': None,
            'svg': None
        }

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
        self.__times['state'] = datetime.now()

    def addPublication(self, pub):
        self.__pubs[pub.name] = pub
        self.__times['state'] = datetime.now()

    def addService(self, srv):
        self.__srvs[srv.name] = srv
        self.__times['state'] = datetime.now()

    def graph(self):
        # Check if we can return an existing graph
        if (self.__graph is not None) and (self.__times['graph'] >= self.__times['state']):
            return self.__graph

        # Remember time
        rospy.loginfo(f'Generating graph for node {self.__name}')
        self.__times['graph'] = datetime.now()

        # Generate new graph
        graph = pgv.AGraph(directed=True, forcelabels=True, stylesheet='https://www.w3schools.com/w3css/4/w3.css')

        # Add node
        graph.add_node(self.__name, **{'shape': 'oval', 'class': 'w3-orange w3-hover-red'})

        # Add topics to which the node subscribes
        node_names = set()
        for topic_name, topic in self.__subs.items():
            topic_id = 'topic_sub_' + topic_name
            node_url = url_for('topic.get_topic_info', name=topic_name)
            node_label = f"{topic_name}\\n{topic.type}"
            graph.add_node(topic_id, label=node_label, shape='box', URL=node_url, target='_top')
            graph.add_edge(topic_id, self.__name)

            for pub_name in topic.publishers:
                subnode_id = 'pub_' + pub_name
                if pub_name not in node_names:
                    subnode_url = url_for('node.get_node_info', name=pub_name)
                    graph.add_node(subnode_id, label=pub_name, shape='oval', URL=subnode_url, target='_top')
                    node_names.add(pub_name)
                graph.add_edge(subnode_id, topic_id)

        # Add topics which the node publishes
        node_names.clear()
        for topic_name, topic in self.__pubs.items():
            topic_id = 'topic_pub_' + topic_name
            node_url = url_for('topic.get_topic_info', name=topic_name)
            node_label = f"{topic_name}\\n{topic.type}"
            graph.add_node(topic_id, label=node_label, shape='box', URL=node_url, target='_top')
            graph.add_edge(self.__name, topic_id)

            for sub_name in topic.subscribers:
                subnode_id = 'sub_' + sub_name
                if sub_name not in node_names:
                    subnode_url = url_for('node.get_node_info', name=sub_name)
                    graph.add_node(subnode_id, label=sub_name, shape='oval', URL=subnode_url, target='_top')
                    node_names.add(sub_name)
                graph.add_edge(topic_id, subnode_id)

        # Store and return graph
        self.__graph = graph
        return graph

    def svg(self):
        # Check if we can return an existing svg
        graph = self.graph()
        if (self.__svg is not None) and (self.__times['svg'] >= self.__times['graph']):
            return self.__svg

        # Remember time
        rospy.loginfo(f'Generating svg for node {self.__name}')
        self.__times['svg'] = datetime.now()

        # Generate svg
        img_stream = BytesIO()
        graph.draw(path=img_stream, format='svg', prog='dot')
        svg = img_stream.getvalue().decode('utf-8')
        svg = Markup(svg.replace('xlink:', '').replace('w3&#45;', 'w3-').replace('hover&#45;', 'hover-'))

        # Store and return svg
        self.__svg = svg
        return svg


class Service:
    def __init__(self, name):
        self.__name = name
        try:
            self.__type = rosservice.get_service_type(name)
            self.__data_class = rosservice.get_service_class_by_name(name)
            self.__data_class_request = roslib.message.get_service_class(self.__type + 'Request')
        except Exception as e:
            rospy.logerr(str(e))
            self.__type = None
            self.__data_class = None
            self.__data_class_request = None
        self.__providers = dict()
        self.__graph = None
        self.__svg = None
        self.__times = {
            'state': datetime.now(),
            'graph': None,
            'svg': None
        }

    @property
    def name(self):
        return self.__name

    @property
    def type(self):
        return self.__type

    @property
    def providers(self):
        return self.__providers

    @property
    def data_class(self):
        return self.__data_class

    @property
    def request_class(self):
        return self.__data_class_request

    def call(self, args):
        try:
            return rospy.ServiceProxy(self.__name, self.__data_class)(args)
        except Exception as e:
            rospy.loginfo(e)
            raise e

    def clear(self):
        self.__providers.clear()
        self.__times['state'] = datetime.now()

    def addProvider(self, node):
        self.__providers[node.name] = node
        self.__times['state'] = datetime.now()

    def graph(self):
        # Check if we can return an existing graph
        if (self.__graph is not None) and (self.__times['graph'] >= self.__times['state']):
            return self.__graph

        # Remember time
        rospy.loginfo(f'Generating graph for service {self.__name}')
        self.__times['graph'] = datetime.now()

        # Generate new graph
        graph = pgv.AGraph(directed=True, forcelabels=True, stylesheet='https://www.w3schools.com/w3css/4/w3.css')
        
        node_label = f"{self.__name}\\n{self.__type}"
        node_id = "srv_" + self.__name
        graph.add_node(node_id, label=node_label, shape='box')

        for _, node in self.__providers.items():
            node_url = url_for('node.get_node_info', name=node.name)
            graph.add_node(node.name, shape='oval', URL=node_url, target='_top')
            graph.add_edge(node.name, node_id)

        # Store and return graph
        self.__graph = graph
        return graph

    def svg(self):
        # Check if we can return an existing svg
        graph = self.graph()
        if (self.__svg is not None) and (self.__times['svg'] >= self.__times['graph']):
            return self.__svg

        # Remember time
        rospy.loginfo(f'Generating svg for node {self.__name}')
        self.__times['svg'] = datetime.now()

        # Generate svg
        img_stream = BytesIO()
        graph.draw(path=img_stream, format='svg', prog='dot')
        svg = img_stream.getvalue().decode('utf-8')
        svg = Markup(svg.replace('xlink:', '').replace('w3&#45;', 'w3-').replace('hover&#45;', 'hover-'))

        # Store and return svg
        self.__svg = svg
        return svg


class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class ROSApi(metaclass=Singleton):
    def __init__(self):
        rospy.loginfo("Instance created")
        self.__system_state = None
        self.__nodes = dict()
        self.__topics = dict()
        self.__services = dict()
        self.__params = dict()
        self.__blacklisted_topics = set()
        self.__blacklisted_nodes = set()
        self.__graph = None
        self.__svg = None
        self.__master_pid = None
        self.__times = {
            'state': datetime.now(),
            'graph': None,
            'svg': None
        }

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

    def graph(self):
        # Check if we can return an existing graph
        if (self.__graph is not None) and (self.__times['graph'] >= self.__times['state']):
            return self.__graph

        # Remember time
        rospy.loginfo(f'Generating overview graph')
        self.__times['graph'] = datetime.now()

        # Generate new graph
        graph = pgv.AGraph(directed=True, forcelabels=True)

        # Add one graph node for each ros node
        for node_name in ros.nodes:
            node_url = url_for('node.get_node_info', name=node_name)
            graph.add_node(node_name, shape='oval', URL=node_url, target='_top')
            
        # Iterate over topics, add a graph node for each topic and draw edges to subscribers and publishers
        for topic_name, topic in ros.topics.items():
            topic_id = 'topic_' + topic_name
            node_url = url_for('topic.get_topic_info', name=topic_name)
            node_label = f"{topic_name}\\n{topic.type}"
            graph.add_node(topic_id, label=node_label, shape='box', URL=node_url, target='_top')

            for node_name in topic.publishers:
                graph.add_edge(node_name, topic_id)

            for node_name in topic.subscribers:
                graph.add_edge(topic_id, node_name)

        # Store and return graph
        self.__graph = graph
        return graph

    def svg(self):
        # Check if we can return an existing svg
        graph = self.graph()
        if (self.__svg is not None) and (self.__times['svg'] >= self.__times['graph']):
            return self.__svg

        # Remember time
        rospy.loginfo(f'Generating overview svg')
        self.__times['svg'] = datetime.now()

        # Generate svg
        img_stream = BytesIO()
        graph.draw(path=img_stream, format='svg', prog='dot')
        svg = img_stream.getvalue().decode('utf-8')
        svg = Markup(svg.replace('xlink:', '').replace('w3&#45;', 'w3-').replace('hover&#45;', 'hover-'))

        # Store and return svg
        self.__svg = svg
        return svg

    def get_topic(self, name):
        if name in self.__topics:
            return self.__topics[name]
        if '/'+name in self.__topics:
            return self.__topics['/'+name]
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
            pid = master.getPid()
            state = master.getSystemState()
        except socket.error:
            pid = None
            raise rosnode.ROSNodeIOException("Unable to communicate with master!")
        
        try:
            param_names = rosparam.list_params('')
        except rosparam.RosParamIOException:
            rospy.loginfo('Could not fetch parameter names from server')
        self.__params = sorted(set(['/'.join(param.split('/')[:2]) for param in param_names]))

        if state is None or state != self.__system_state or pid != self.__master_pid:
            # Clear state
            rospy.loginfo('Resetting nodes and services')
            self.__times['state'] = datetime.now()
            
            # Topics are not deleted so that we don't loose the message statistics
            self.__nodes.clear()
            self.__services.clear()
            if pid != self.__master_pid:
                rospy.loginfo("Resetting topics since roscore PID has changed")
                self.__master_pid = pid
                topic_names = []
                for name, topic in self.__topics.items():
                    topic_names.append(name)
                    topic.unregister()
                for name in topic_names:
                    del self.__topics[name]
            else:    
                for _, topic in self.__topics.items():
                    topic.clear()

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

            # Store system state
            self.__system_state = state

ros = ROSApi()
#rospy.init_node('ros_web_gui')
#_thread.start_new_thread(rospy.spin, ()) 
