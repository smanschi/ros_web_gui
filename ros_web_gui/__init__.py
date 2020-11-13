from flask import Flask, Markup, Response, render_template, url_for
from . import menu, node, service, topic, param
from .ros import ros

def create_app(test_config=None):
    import pygraphviz as pgv
    import os
    import socket
    import yaml
    from . import menu
    from .ros import ros
    from io import BytesIO

    app = Flask(__name__)

    app.register_blueprint(node.bp)
    app.register_blueprint(service.bp)
    app.register_blueprint(topic.bp)
    app.register_blueprint(param.bp)

    # Read config file
    config_file = os.path.join(os.path.basename(__file__), '..', 'config', 'config.yaml')
    with open(config_file, 'r') as stream:
    try:
        config = yaml.safe_load(stream))
    except yaml.YAMLError as exc:
        print(exc)
        
    # Configure ROS api
    ros.config(**config)

    def get_graph():
        graph = pgv.AGraph(directed=True, forcelabels=True)
        node_names = set()

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

        return graph

    @app.route('/graph.svg')
    def get_graph_svg():
        # Update ros api
        ros.update()

        # Generate graph
        graph = get_graph()
        img_stream = BytesIO()
        graph.draw(path=img_stream, format='svg', prog='dot')
        svg = img_stream.getvalue().decode('utf-8')
        svg = Markup(svg.replace('xlink:', '').replace('w3&#45;', 'w3-').replace('hover&#45;', 'hover-'))
        return Response(svg, mimetype='image/svg+xml')

    @app.route('/')
    @app.route('/index.html')
    def get_main():
        # Update ros api
        ros.update()

        # Empty content at the moment
        content = ''

        # Link to svg
        img_data = 'graph.svg'

        # Render and return template
        return render_template('base_with_svg.html', title='Overview',
                               active_menu_item='home',
                               content=content, **menu.get_items(),
                               img_data=img_data)

    return app
