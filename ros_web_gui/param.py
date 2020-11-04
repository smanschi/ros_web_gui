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

def get_info(name):
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
    type_str = 'Type: ' + type(content).__name__ + '\n'
    if type(content) in [dict, list]:
        content = type_str + '\n' + yaml.dump(content)
    else:
        content = type_str + '\n' + str(content)

    # Format param info
    content = Markup('<div class="w3-container">' + content.replace('\n', '<br/>') + '</div>')

    # Current url
    url = url_for('param.get_param_info', name=name)

    # Return rendered template
    return render_template('base.html', title=f'Parameter {name}',
                            active_menu_item='param',
                            content=content,
                            **menu.get_items(data, active_item=url))

@bp.route('/')
def get_default_info():
    return get_info('/')

@bp.route('/<path:name>')
def get_param_info(name):
    return get_info(name)
    