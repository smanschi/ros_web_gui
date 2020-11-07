from flask import Blueprint, Markup, Response, render_template, url_for
from . import menu, param, ros
import base64
from io import BytesIO, StringIO
import pygraphviz as pgv
import rosgraph
import rosgraph.names
import rosnode
import socket
import sys
import urllib

bp = Blueprint('node', __name__, url_prefix='/node')

def get_node_info_description(data, node):
    buff = ""

    num_pubs = 0
    for d in data['pubs']:
        if node in d['publisher']:
            if num_pubs == 0:
                buff += "\nPublications: \n"
            buff += f"* {d['topic']} [{d['type']}]\n"
            num_pubs += 1
    if num_pubs == 0:
        buff += "\nPublications: None\n"

    num_subs = 0
    for d in data['subs']:
        if node in d['subscriber']:
            if num_subs == 0:
                buff += "\nSubscriptions: \n"
            buff += f"* {d['topic']} [{d['type']}]\n"
            num_subs += 1
    if num_subs == 0:
        buff += "\nSubscriptions: None\n"

    num_srvs = 0
    for d in data['srvs']:
        if node in d['provider']:
            if num_srvs == 0:
                buff += "\nServices: \n"
            buff += f"* {d['topic']} [{d['type']}]\n"
            num_srvs += 1
    if num_srvs == 0:
        buff += "\nServices: None\n"
        
    return buff

def get_graph(data, node_name):    
    graph = pgv.AGraph(directed=True, forcelabels=True, stylesheet='https://www.w3schools.com/w3css/4/w3.css')

     # Add node
    #node_url = url_for('node.get_node_info', name=node_name)
    graph.add_node(node_name, **{'shape': 'oval', 'class': 'w3-orange w3-hover-red'})#, URL=node_url, target='_top')

    # Add topics to which the node subscribes
    node_names = set()
    for d in data['subs']:
        if node_name in d['subscriber']:
            topic_id = 'topic_sub_' + d['topic']
            node_url = url_for('topic.get_topic_info', name=d['topic'])
            node_label = f"{d['topic']}\\n{d['type']}"
            graph.add_node(topic_id, label=node_label, shape='box', URL=node_url, target='_top')
            graph.add_edge(topic_id, node_name)

            for e in data['pubs']:
                if e['topic'] == d['topic']:
                    for pub in e['publisher']:
                        subnode_id = 'sub_pub_' + pub
                        if pub not in node_names:
                            subnode_url = url_for('node.get_node_info', name=pub)
                            graph.add_node(subnode_id, label=pub, shape='oval', URL=subnode_url, target='_top')
                            node_names.add(pub)
                        graph.add_edge(subnode_id, topic_id)

    # Add topics which the node publishes
    node_names.clear()
    for d in data['pubs']:
        if node_name in d['publisher']:
            topic_id = 'topic_pub_' + d['topic']
            node_url = url_for('topic.get_topic_info', name=d['topic'])
            node_label = f"{d['topic']}\\n{d['type']}"
            graph.add_node(topic_id, label=node_label, shape='box', URL=node_url, target='_top')
            graph.add_edge(node_name, topic_id)

            for e in data['subs']:
                if e['topic'] == d['topic']:
                    for sub in e['subscriber']:
                        subnode_id = 'pub_sub_' + sub
                        if sub not in node_names:
                            subnode_url = url_for('node.get_node_info', name=sub)
                            graph.add_node(subnode_id, label=sub, shape='oval', URL=subnode_url, target='_top')
                            node_names.add(sub)
                        graph.add_edge(topic_id, subnode_id)

    return graph
    
@bp.route('/')
def get_node_overview():
    # Get info about nodes
    data = ros.get_info()

    # Get menu items
    menu_items = menu.get_items(data)

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

    # Get info about node
    data = ros.get_info()

    if generate_svg is True:
        # Generate graph
        graph = get_graph(data, name)
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
                               **menu.get_items(data, active_item=url),
                               img_data=img_data)
                               #img_data=img_stream)
                               #img_data=urllib.parse.quote(img_stream.rstrip('\n')))
                            