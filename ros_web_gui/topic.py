from flask import Blueprint, Markup, Response, render_template, url_for
from . import menu, ros
from io import BytesIO
import pygraphviz as pgv
import rosgraph

bp = Blueprint('topic', __name__, url_prefix='/topic')

def get_graph(data, topic):
    graph = pgv.AGraph(directed=True, forcelabels=True)
    node_names = set()

    # Add publishers
    for d in data['pubs']:
        if d['topic'] == topic:
            topic_id = 'topic_' + d['topic']
            if topic_id not in node_names:
                #node_url = url_for('topic.get_topic_info', name=topic)
                node_label = f"{d['topic']}\\n{d['type']}"
                graph.add_node(topic_id, label=node_label, shape='box')#, URL=node_url, target='_top')
                node_names.add(topic_id)

            for pub in d['publisher']:
                if pub not in node_names:
                    node_url = url_for('node.get_node_info', name=pub)
                    graph.add_node(pub, shape='oval', URL=node_url, target='_top')
                    node_names.add(pub)
                graph.add_edge(pub, topic_id)

    # Add subscribers
    for d in data['subs']:
        if d['topic'] == topic:
            topic_id = 'topic_' + d['topic']
            if topic_id not in node_names:
                #node_url = url_for('topic.get_topic_info', name=topic)
                node_label = f"{d['topic']}\\n{d['type']}"
                graph.add_node(topic_id, label=node_label, shape='box')#, URL=node_url, target='_top')

            for sub in d['subscriber']:
                if sub not in node_names:
                    node_url = url_for('node.get_node_info', name=sub)
                    graph.add_node(sub, shape='oval', URL=node_url, target='_top')
                    node_names.add(sub)
                graph.add_edge(topic_id, sub)

    return graph

@bp.route('/')
def get_topic_overview():
    # Get info about nodes
    data = ros.get_info()

    # Get menu items
    menu_items = menu.get_items(data)

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

    # Resolve topic name
    #topic_name = rosgraph.names.script_resolve_name('rostopic', name)
    topic_name = name

    # Get info about topic
    data = ros.get_info()
    #topic = get_topic(topic_name)

    if generate_svg is True:
        # Generate graph
        graph = get_graph(data, name)
        img_stream = BytesIO()
        graph.draw(path=img_stream, format='svg', prog='dot')
        svg = img_stream.getvalue().decode('utf-8')
        svg = svg.replace('xlink:', '')

        return Response(svg, mimetype='image/svg+xml')
    else:
        content = ''  # get_topic_info_description(topic)

        # Format topic info
        content = Markup(content.replace('\n', '<br/>'))

        # Image url
        url = url_for('topic.get_topic_info', name=name)
        img_data = url + '.svg'

        # Return rendered template
        return render_template('base_with_svg.html', title=f'Topic {topic_name}',
                               active_menu_item='topic',
                               content=content,
                               **menu.get_items(data, active_item=url),
                               img_data=img_data)
                               #img_data=img_stream)
                               #img_data=urllib.parse.quote(img_stream.rstrip('\n')))
                            