from .ros import ros
from flask import url_for

def get_items(active_item=None):
    # Navigation
    nav_node_items =    [{'href': url_for('node.get_node_info', name=node), 'caption': node} for node in sorted(ros.nodes)]
    nav_topic_items =   [{'href': url_for('topic.get_topic_info', name=topic), 'caption': topic} for topic in sorted(ros.topics)]
    nav_service_items = [{'href': url_for('service.get_service_info', name=service), 'caption': service} for service in sorted(ros.services)]
    nav_param_items =   [{'href': url_for('param.get_param_info', name=param), 'caption': param} for param in sorted(ros.params)]

    if active_item is not None:
        nav_node_items = [{**item, 'active': True if item['href'] == active_item else False} for item in nav_node_items]
        nav_topic_items = [{**item, 'active': True if item['href'] == active_item else False} for item in nav_topic_items]
        nav_service_items = [{**item, 'active': True if item['href'] == active_item else False} for item in nav_service_items]
        nav_param_items = [{**item, 'active': True if item['href'] == active_item else False} for item in nav_param_items]

    return {
        'nav_node_items': nav_node_items,
        'nav_topic_items': nav_topic_items,
        'nav_service_items': nav_service_items,
        'nav_param_items': nav_param_items
    }
