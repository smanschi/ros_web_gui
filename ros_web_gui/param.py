from flask import Blueprint, Markup, Response, render_template, url_for
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

@bp.route('/')
def get_param_overview():
    # Get info about nodes
    data = ros.get_info()

    # Get menu items
    menu_items = menu.get_items(data)

    # Iterate over nodes
    content = ''
    items = menu_items['nav_param_items']

    # Return rendered template
    return render_template('base_with_list.html', title=f'List of Parameters',
                            active_menu_item='param',
                            content=content,
                            items=items,
                            **menu_items)

def get_text(content):
    buff = ''
    if type(content) == list:
        #buff += '<ul style="list-style-type:none;">\n'
        for ele in content:
            #buff += '<li>'
            buff += get_text(ele)
            #buff += '</li>\n'
        #buff += '</ul>\n'
    elif type(content) == dict:
        for key, val in content.items():
            buff += key + ':\n'
            buff += '<div style="display:inline-block; vertical-align: top">\n'
            buff += get_text(val)
            buff += '</div><br />\n'
    else:
        buff += str(content) + '<br />'
    return buff

@bp.route('/<path:name>')
def get_param_info(name):
    if not name.startswith('/'):
        name = '/' + name

    # Get info about param
    data = ros.get_info()

    # Generate content
    try:
        content = rosparam.get_param(name)
    except rosparam.RosParamIOException:
        content = f'Could net get value of {name} from parameter server'

    # Type handling
    content = get_text(content)

    # Format param info
    content = Markup('<div class="w3-container">' + content + '</div>')

    # Current url
    url = url_for('param.get_param_info', name=name)

    # Return rendered template
    return render_template('base.html', title=f'Parameter {name}',
                            active_menu_item='param',
                            content=content,
                            **menu.get_items(data, active_item=url))
                            