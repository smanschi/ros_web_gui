from flask import url_for
import rosnode
import rosservice
import rostopic

def get_items(data, active_item=None):
    # Search for node, topic and service names
    node_names = set()
    topic_names = set()
    service_names = set()
    for d in data['pubs']:
        topic_names.add(d['topic'])
        for node in d['publisher']:
            node_names.add(node)
    for d in data['subs']:
        topic_names.add(d['topic'])
        for node in d['subscriber']:
            node_names.add(node)
    for d in data['srvs']:
        service_names.add(d['topic'])
        for node in d['provider']:
            node_names.add(node)

    # Navigation
    nav_node_items =    [{'href': url_for('node.get_node_info', name=node), 'caption': node} for node in sorted(node_names)]
    nav_topic_items =   [{'href': url_for('topic.get_topic_info', name=topic), 'caption': topic} for topic in sorted(topic_names)]
    nav_service_items = [{'href': url_for('service.get_service_info', name=service), 'caption': service} for service in sorted(service_names)]

    if active_item is not None:
        nav_node_items = [{**item, 'active': True if item['href'] == active_item else False} for item in nav_node_items]
        nav_topic_items = [{**item, 'active': True if item['href'] == active_item else False} for item in nav_topic_items]
        nav_service_items = [{**item, 'active': True if item['href'] == active_item else False} for item in nav_service_items]


    return {
        'nav_node_items': nav_node_items,
        'nav_topic_items': nav_topic_items,
        'nav_service_items': nav_service_items
    }
