from flask import Blueprint, Markup, Response, render_template, url_for
from . import menu
from .ros import ros
from io import BytesIO, StringIO
import pygraphviz as pgv
import rosservice
import sys

bp = Blueprint('service', __name__, url_prefix='/service')

def get_graph(service):    
    graph = pgv.AGraph(directed=True, forcelabels=True, stylesheet='https://www.w3schools.com/w3css/4/w3.css')
    
    node_label = f"{service.name}\\n{service.type}"
    node_id = "srv_" + service.name
    graph.add_node(node_id, label=node_label, shape='box')

    for _, node in service.providers.items():
        node_url = url_for('node.get_node_info', name=node.name)
        graph.add_node(node.name, shape='oval', URL=node_url, target='_top')
        graph.add_edge(node.name, node_id)

    return graph

@bp.route('/')
def get_service_overview():
    # Update ros api
    ros.update()

    # Get menu items
    menu_items = menu.get_items()

    # Iterate over nodes
    content = ''
    items = menu_items['nav_service_items']

    # Return rendered template
    return render_template('base_with_list.html', title=f'List of Services',
                            active_menu_item='service',
                            content=content,
                            items=items,
                            **menu_items)   

@bp.route('/<path:name>')
def get_service_info(name):
    # Check if a svg representation should be returned
    generate_svg = False 
    if name.endswith('.svg'):
        name = name[:-4]
        generate_svg = True

    if not name.startswith('/'):
        name = '/' + name

    # Update ros api
    ros.update()

    # Get service
    service = ros.get_service(name)

    if generate_svg is True:
        # Generate graph
        graph = get_graph(service)
        img_stream = BytesIO()
        graph.draw(path=img_stream, format='svg', prog='dot')
        svg = img_stream.getvalue().decode('utf-8')
        svg = Markup(svg.replace('xlink:', '').replace('w3&#45;', 'w3-').replace('hover&#45;', 'hover-'))

        return Response(svg, mimetype='image/svg+xml')
    else:
        # # Get service info
        # old_stdout = sys.stdout
        # sys.stdout = mystdout = StringIO()
        # rosservice._rosservice_info(name)
        # content = mystdout.getvalue()
        # sys.stdout = old_stdout
        # mystdout.close()
        content = ''

        # Format node info
        content = Markup(content.replace('\n', '<br/>'))

        # Link to svg
        url = url_for('service.get_service_info', name=name)
        img_data = url + '.svg'

        # Return rendered template
        return render_template('base_with_svg.html', title=f'Service {name}',
                               active_menu_item='service',
                               content=content,
                               **menu.get_items(active_item=url),
                               img_data=img_data)
                               #img_data=img_stream)
                               #img_data=urllib.parse.quote(img_stream.rstrip('\n')))