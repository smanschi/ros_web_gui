from flask import Blueprint, Markup, Response, escape, render_template, url_for
from . import menu, ros
import base64
from io import BytesIO, StringIO
import pygraphviz as pgv
import rosparam
import socket
import sys
import urllib
import yaml

bp = Blueprint('param', __name__, url_prefix='/param')

def get_text(content, namespace='/'):
    buff = ''
    if type(content) == list:
        for idx, ele in enumerate(content):
            if type(ele) is dict:
                buff += get_text(ele, namespace + f'[{idx}]/')
            else:
                buff += get_text(ele, namespace + f'[{idx}]')
    elif type(content) == dict:
        for key, val in content.items():
            if type(val) is not dict:
                new_namespace = namespace + f'{key}'
            else:
                new_namespace = namespace + f'{key}/'
            buff += get_text(val, new_namespace)
    else:
        buff += f'{namespace}<br />'
        if type(content) == str and len(content) > 5 and content[:5] == '<?xml':
            buff +=  f'<pre style="display: inline-block"><code class="html">{escape(content)}</code></pre><br />\n'
        else:
            buff +=  f'<pre style="display: inline-block"><code class="plaintext">{escape(content)}</code></pre><br />\n'
    return buff

def get_param(name):
    # Generate content
    try:
        content = rosparam.get_param(name)
    except rosparam.RosParamIOException:
        content = f'Could net get value of {name} from parameter server'

    # Type handling
    content = get_text(content, namespace=name)

    # Format param info
    content = Markup('<div class="w3-container">' + content + '</div>')
    return content

# @bp.route('/')
# def get_param_overview():
#     # Get info about nodes
#     data = ros.get_info()

#     # Get menu items
#     menu_items = menu.get_items(data)

#     # Iterate over nodes
#     content = ''
#     items = menu_items['nav_param_items']

#     # Return rendered template
#     return render_template('base_with_list.html', title=f'List of Parameters',
#                             active_menu_item='param',
#                             content=content,
#                             items=items,
#                             **menu_items)

@bp.route('/')
def get_param_overview():
    # Get info about nodes
    data = ros.get_info()

    # Get menu items
    menu_items = menu.get_items(data)

    # Iterate over nodes
    content = get_param('/')

    # Return rendered template
    return render_template('base.html', title=f'List of Parameters',
                            active_menu_item='param',
                            content=content,
                            **menu_items)

@bp.route('/<path:name>')
def get_param_info(name):
    if not name.startswith('/'):
        name = '/' + name

    # Get info about param
    data = ros.get_info()

    # Generate content
    content = get_param(name)

    # Current url
    url = url_for('param.get_param_info', name=name)

    # Return rendered template
    return render_template('base.html', title=f'Parameter {name}',
                            active_menu_item='param',
                            content=content,
                            **menu.get_items(data, active_item=url))
                            