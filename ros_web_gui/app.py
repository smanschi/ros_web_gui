from flask import Flask, Markup, Response, render_template, url_for
from . import menu, node, service, topic, param
from .ros import ros

def create_app(config=None):
    import pygraphviz as pgv
    import os
    import rospy
    import signal
    import socket
    import sys
    import yaml
    from . import menu
    from .ros import ros
    from io import BytesIO

    # Signal handler (don't know why this is needed)
    def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Read config file
    if config is None:
        config_file = os.path.join(os.path.dirname(__file__), 'config.yaml')
        with open(config_file, 'r') as stream:
            try:
                config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

    # Instantiate ROS api
    rospy.init_node('ros_web_gui')
    if 'blacklisted_nodes' not in config:
        config['blacklisted_nodes'] = ['ros_web_gui']
    else:
        config['blacklisted_nodes'].append('ros_web_gui')
        
    # Configure ROS api
    ros.config(**config)

    # Start ROS api
    ros.update()

    # Assemble app
    app = Flask(__name__)
    app.register_blueprint(node.bp)
    app.register_blueprint(service.bp)
    app.register_blueprint(topic.bp)
    app.register_blueprint(param.bp)

    @app.route('/graph.svg')
    def get_graph_svg():
        # Update ros api
        ros.update()

        # Get svg
        svg = ros.svg()
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