from flask import Blueprint, Markup, Response, jsonify, render_template, request, url_for
from . import menu
from .ros import ros
from io import BytesIO
import pygraphviz as pgv
import rosgraph

bp = Blueprint('topic', __name__, url_prefix='/topic')

@bp.route('/')
def get_topic_overview():
    # Update ros api
    ros.update()

    # Get menu items
    menu_items = menu.get_items()

    # Iterate over nodes
    content = ''
    items = menu_items['nav_topic_items']

    # Return rendered template
    return render_template('base_with_list.html', title=f'List of Topics',
                            active_menu_item='topic',
                            content=content,
                            items=items,
                            **menu_items)   

@bp.route('/<path:name>', methods=['GET', 'POST'])
def get_topic_info(name):
    # Check if a json or svg representation should be returned
    mode = request.args.get('get', '', type=str)

    # Update ros api
    ros.update()

    # Add slash to name
    if name[0] != '/':
        name_without_slash = name
        name = '/' + name
    else:
        name_without_slash = name[1:]

    # Get topic
    topic = ros.get_topic(name)

    # Handle post request
    if request.method == 'POST':
        msg_pub = request.form['msg_pub']
        response = {
            'success': True,
            'error_msg': None
        }
        try:
            topic.publish(msg_pub)
        except Exception as e:
            response['success'] = False
            response['error_msg'] = str(e)
        return jsonify(response)

    # Return svg
    if mode == 'svg':
        svg = topic.svg()
        return Response(svg, mimetype='image/svg+xml')

    # Get message
    msg = topic.msg
    if msg is not None:
        msg_data = [
            {'id': 'num_messages', 'description': 'Messages received', 'value': msg['num_messages']},
            {'id': 'messages_per_second', 'description': 'Messages per second', 'value': f"{msg['messages_per_second']:.1f}"},
            {'id': 'avg_size', 'description': 'Average message size', 'value': f"{msg['avg_size']:.0f} Bytes"},
            {'id': 'bandwidth', 'description': 'Bandwidth', 'value': f"{msg['avg_size']*msg['messages_per_second']:.0f} Bytes/s"},
            {'id': 'last_message', 'description': 'Last message received', 'value': msg["last_message"].strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]},
            {'id': 'msg', 'description': 'Message', 'value': str(msg['msg'])}
        ]           
    else:
        msg_data = [
            {'id': 'num_messages', 'description': 'Messages received', 'value': 0},
            {'id': 'messages_per_second', 'description': 'Messages per second', 'value': '-'},
            {'id': 'avg_size', 'description': 'Average message size', 'value': '-'},
            {'id': 'bandwidth', 'description': 'Bandwidth', 'value': '-'},
            {'id': 'last_message', 'description': 'Last message received', 'value': '-'},
            {'id': 'msg', 'description': 'Message', 'value': 'None'}
        ]

    if mode == 'json':
        print(f'Returning JSON for topic {topic.name}')
        return jsonify(msg_data)
    else:
        content = ''  # get_topic_info_description(topic)

        # Format topic info
        content = Markup(content.replace('\n', '<br/>'))

        # Image url
        url = url_for('topic.get_topic_info', name=name_without_slash)
        img_data = url + '?get=svg'

        # Return rendered template
        return render_template('topic.html', title=f'Topic {name}',
                               active_menu_item='topic',
                               url=url,
                               msg_items=msg_data,
                               msg=None if msg is None else msg['msg'],
                               msg_template=topic.msg_template(),
                               content=content,
                               **menu.get_items(active_item=url),
                               img_data=img_data)
                            