from flask import Blueprint, Markup, Response, render_template, request, url_for
from . import menu, param
from .ros import *
import sys

bp = Blueprint('node', __name__, url_prefix='/node')

@bp.route('/')
def get_node_overview():
    # Update ros api
    ros.update()

    # Get menu items
    menu_items = menu.get_items()

    # Get list of nodes
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
    generate_svg = request.args.get('get', '', type=str) == 'svg'

    if not name.startswith('/'):
        name = '/' + name

    # Update ros api
    ros.update()

    # Get node
    node = ros.get_node(name)

    if generate_svg is True:
        # Generate graph
        svg = node.svg()
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
        img_data = url + '?get=svg'

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
                            