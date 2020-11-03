from flask import Blueprint, Markup, Response, render_template, url_for
from . import menu, ros
import base64
from io import BytesIO, StringIO
import pygraphviz as pgv
import rosparam
import socket
import sys
import urllib

bp = Blueprint('param', __name__, url_prefix='/param')

@bp.route('/<path:name>', methods=('GET', 'POST'))
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

    # Format param info
    content = Markup('<div class="w3-container">' + content.replace('\n', '<br/>') + '</div>')

    # Current url
    url = url_for('param.get_param_info', name=name)

    # Return rendered template
    return render_template('base.html', title=f'Parameter {name}',
                            active_menu_item='param',
                            content=content,
                            **menu.get_items(data, active_item=url))
                            