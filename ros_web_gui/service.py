from flask import Blueprint, Markup, Response, render_template, url_for
from . import menu, ros
from io import BytesIO, StringIO
import pygraphviz as pgv
import rosservice
import sys

bp = Blueprint('service', __name__, url_prefix='/service')

def get_graph(data, service_name):    
    graph = pgv.AGraph(directed=True, forcelabels=True, stylesheet='https://www.w3schools.com/w3css/4/w3.css')
    
    # Add publishers of service
    for d in data['srvs']:
        if service_name == d['topic']:
            service_id = 'srv_' + d['topic']
            node_label = f"{d['topic']}\\n{d['type']}"
            graph.add_node(service_id, label=node_label, shape='box')
            for provider in d['provider']:
                subnode_id = 'prov_' + provider
                subnode_url = url_for('node.get_node_info', name=provider)
                graph.add_node(subnode_id, label=provider, shape='oval', URL=subnode_url, target='_top')
                graph.add_edge(subnode_id, service_id)

    return graph

@bp.route('/<path:name>', methods=('GET', 'POST'))
def get_service_info(name):
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
                               **menu.get_items(data, active_item=url),
                               img_data=img_data)
                               #img_data=img_stream)
                               #img_data=urllib.parse.quote(img_stream.rstrip('\n')))