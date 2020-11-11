from flask import Blueprint, Markup, Response, render_template, url_for
from . import menu, param
from .ros import *
import base64
from io import BytesIO, StringIO
import pygraphviz as pgv
import socket
import sys
import urllib

bp = Blueprint('node', __name__, url_prefix='/node')


def get_graph(node):    
    graph = pgv.AGraph(directed=True, forcelabels=True, stylesheet='https://www.w3schools.com/w3css/4/w3.css')

     # Add node
    #node_url = url_for('node.get_node_info', name=node_name)
    graph.add_node(node.name, **{'shape': 'oval', 'class': 'w3-orange w3-hover-red'})#, URL=node_url, target='_top')

    # Add topics to which the node subscribes
    node_names = set()
    for topic_name, topic in node.subscriptions.items():
        topic_id = 'topic_sub_' + topic_name
        node_url = url_for('topic.get_topic_info', name=topic_name)
        node_label = f"{topic_name}\\n{topic.type}"
        graph.add_node(topic_id, label=node_label, shape='box', URL=node_url, target='_top')
        graph.add_edge(topic_id, node.name)

        for pub_name in topic.publishers:
            subnode_id = 'pub_' + pub_name
            if pub_name not in node_names:
                subnode_url = url_for('node.get_node_info', name=pub_name)
                graph.add_node(subnode_id, label=pub_name, shape='oval', URL=subnode_url, target='_top')
                node_names.add(pub_name)
            graph.add_edge(subnode_id, topic_id)

    # Add topics which the node publishes
    node_names.clear()
    for topic_name, topic in node.publications.items():
        topic_id = 'topic_pub_' + topic_name
        node_url = url_for('topic.get_topic_info', name=topic_name)
        node_label = f"{topic_name}\\n{topic.type}"
        graph.add_node(topic_id, label=node_label, shape='box', URL=node_url, target='_top')
        graph.add_edge(node.name, topic_id)

        for sub_name in topic.subscribers:
            subnode_id = 'sub_' + sub_name
            if sub_name not in node_names:
                subnode_url = url_for('node.get_node_info', name=sub_name)
                graph.add_node(subnode_id, label=sub_name, shape='oval', URL=subnode_url, target='_top')
                node_names.add(sub_name)
            graph.add_edge(topic_id, subnode_id)

    return graph
    
@bp.route('/')
def get_node_overview():
    # Update ros api
    ros.update()

    # Get menu items
    menu_items = menu.get_items()

    # Iterate over nodes
    content = ''
    items = menu_items['nav_node_items']

    # Return rendered template
    return render_template('base_with_list.html', title=f'List of Nodes',
                            active_menu_item='node',
                            content=content,
                            items=items,
                            **menu_items)    

@bp.route('/<path:name>')
def get_node_info(name):
    # Check if a svg representation should be returned
    generate_svg = False 
    if name.endswith('.svg'):
        name = name[:-4]
        generate_svg = True

    if not name.startswith('/'):
        name = '/' + name

    # Update ros api
    ros.update()

    # Get node
    node = ros.get_node(name)

    if generate_svg is True:
        # Generate graph
        graph = get_graph(node)
        img_stream = BytesIO()
        graph.draw(path=img_stream, format='svg', prog='dot')
        svg = img_stream.getvalue().decode('utf-8')
        svg = Markup(svg.replace('xlink:', '').replace('w3&#45;', 'w3-').replace('hover&#45;', 'hover-'))

        return Response(svg, mimetype='image/svg+xml')
    else:
        # Assemble content
        content = ''
        param_content = param.get_param(name)
        if param_content is not None:
            content += Markup('<h2>Parameter</h2>')
            content += param.get_param(name)

        # Link to svg
        url = url_for('node.get_node_info', name=name)
        img_data = url + '.svg'

        # # Embed image directly as svg in object
        # For some reason the browsers seem to block the links with this version
        # graph = get_graph(data, name)
        # img_stream = BytesIO()
        # graph.draw(path=img_stream, format='svg', prog='dot')
        # svg = img_stream.getvalue().decode('utf-8')
        # svg = svg.replace('xlink:', '')
        # message_bytes = svg.encode('ascii')
        # base64_bytes = base64.b64encode(message_bytes)
        # base64_message = base64_bytes.decode('ascii')
        # img_data = 'data:image/svg+xml;base64,' + base64_message

        # Return rendered template
        return render_template('base_with_svg.html', title=f'Node {name}',
                               active_menu_item='node',
                               content=content,
                               **menu.get_items(active_item=url),
                               img_data=img_data)
                               #img_data=img_stream)
                               #img_data=urllib.parse.quote(img_stream.rstrip('\n')))
                            