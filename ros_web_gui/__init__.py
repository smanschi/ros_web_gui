from flask import Flask, Markup, Response, render_template, url_for
from . import menu, node, service, topic, ros

def create_app(test_config=None):
    import pygraphviz as pgv
    import rosgraph
    import rosnode
    import rosservice
    import rostopic
    import socket
    from . import menu, node, service
    from io import BytesIO

    app = Flask(__name__)

    app.register_blueprint(node.bp)
    app.register_blueprint(service.bp)
    app.register_blueprint(topic.bp)

    def get_graph(data):
        graph = pgv.AGraph(directed=True, forcelabels=True)
        node_names = set()

        # Add subscribers
        for d in data['subs']:
            topic_id = 'topic_' + d['topic']
            node_url = url_for('topic.get_topic_info', name=d['topic'])
            node_label = f"{d['topic']}\\n{d['type']}"
            graph.add_node(topic_id, label=node_label, shape='box', URL=node_url, target='_top')
            node_names.add(topic_id)

            for sub in d['subscriber']:
                if sub not in node_names:
                    node_url = url_for('node.get_node_info', name=sub)
                    graph.add_node(sub, shape='oval', URL=node_url, target='_top')
                    node_names.add(sub)
                graph.add_edge(topic_id, sub)

        # Add publishers
        for d in data['pubs']:
            topic_id = 'topic_' + d['topic']
            if topic_id not in node_names:
                node_url = url_for('topic.get_topic_info', name=d['topic'])
                node_label = f"{d['topic']}\\n{d['type']}"
                graph.add_node(topic_id, label=node_label, shape='box', URL=node_url, target='_top')
                node_names.add(topic_id)

            for pub in d['publisher']:
                if pub not in node_names:
                    node_url = url_for('node.get_node_info', name=pub)
                    graph.add_node(pub, shape='oval', URL=node_url, target='_top')
                    node_names.add(pub)
                graph.add_edge(pub, topic_id)

        return graph

    @app.route('/graph.svg')
    def get_graph_svg():
        # Generate graph
        graph = get_graph(ros.get_info())
        img_stream = BytesIO()
        graph.draw(path=img_stream, format='svg', prog='dot')
        svg = img_stream.getvalue().decode('utf-8')
        svg = Markup(svg.replace('xlink:', '').replace('w3&#45;', 'w3-').replace('hover&#45;', 'hover-'))
        return Response(svg, mimetype='image/svg+xml')

    @app.route('/')
    @app.route('/index.html')
    def get_main():
        # Empty content at the moment
        content = ''

        # Get info
        data = ros.get_info()

        # Link to svg
        img_data = 'graph.svg'

        # Render and return template
        return render_template('base_with_svg.html', title='Overview',
                               active_menu_item='home',
                               content=content, **menu.get_items(data),
                               img_data=img_data)

    return app