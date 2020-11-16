from flask import Blueprint, Markup, Response, render_template, url_for
from . import menu
from .ros import ros
from io import BytesIO
import pygraphviz as pgv
import rosgraph

bp = Blueprint('topic', __name__, url_prefix='/topic')

def get_graph(topic):
    graph = pgv.AGraph(directed=True, forcelabels=True)

    topic_id = 'topic_' + topic.name
    topic_label = f"{topic.name}\\n{topic.type}"
    graph.add_node(topic_id, label=topic_label, shape='box')

    # Add publishers
    for node_name in topic.publishers:
        node_id = 'pub_' + node_name
        node_url = url_for('node.get_node_info', name=node_name)
        graph.add_node(node_id, label=node_name, shape='oval', URL=node_url, target='_top')
        graph.add_edge(node_id, topic_id)

    # Add subscribers
    for node_name in topic.subscribers:
        node_id = 'sub_' + node_name
        node_url = url_for('node.get_node_info', name=node_name)
        graph.add_node(node_id, label=node_name, shape='oval', URL=node_url, target='_top')
        graph.add_edge(topic_id, node_id)

    return graph

@bp.route('/')
def get_topic_overview():
    # Update ros api
    ros.update()

    # Get menu items
    menu_items = menu.get_items()

    # Iterate over nodes
    content = ''
    items = menu_items['nav_topic_items']

    # Return rendered template
    return render_template('base_with_list.html', title=f'List of Topics',
                            active_menu_item='topic',
                            content=content,
                            items=items,
                            **menu_items)   

@bp.route('/<path:name>')
def get_topic_info(name):
    # Check if a svg representation should be returned
    generate_svg = False 
    if name.endswith('.svg'):
        name = name[:-4]
        generate_svg = True

    if not name.startswith('/'):
        name = '/' + name

    # Update ros api
    ros.update()

    # Get topic
    topic = ros.get_topic(name)

    if generate_svg is True:
        # Generate graph
        graph = get_graph(topic)
        img_stream = BytesIO()
        graph.draw(path=img_stream, format='svg', prog='dot')
        svg = img_stream.getvalue().decode('utf-8')
        svg = svg.replace('xlink:', '')

        return Response(svg, mimetype='image/svg+xml')
    else:
        content = ''  # get_topic_info_description(topic)

        # Get message
        msg = topic.msg
        if msg is not None:
            msg_dict = {
                'Messages received': msg['num_messages'],
                'Messages per second': f"{msg['messages_per_second']:.1f}",
                'Last message received': msg["last_message"].strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            }              
        else:
            msg_dict = {
                'Messages received': 0,
                'Messages per second': "-",
                'Last message': "-"                
            }

        # Format topic info
        content = Markup(content.replace('\n', '<br/>'))

        # Image url
        url = url_for('topic.get_topic_info', name=name)
        img_data = url + '.svg'

        # Return rendered template
        return render_template('topic.html', title=f'Topic {name}',
                               active_menu_item='topic',
                               msg_items=msg_dict,
                               msg=None if msg is None else msg['msg'],
                               content=content,
                               **menu.get_items(active_item=url),
                               img_data=img_data)
                               #img_data=img_stream)
                               #img_data=urllib.parse.quote(img_stream.rstrip('\n')))
                            