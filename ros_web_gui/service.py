from flask import Blueprint, Markup, Response, jsonify, render_template, request, url_for
from . import menu
from .ros import ros
from .util import str_to_msg, initialize_msg
import rosservice
import sys

bp = Blueprint('service', __name__, url_prefix='/service')

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

@bp.route('/<path:name>', methods=['GET', 'POST'])
def get_service_info(name):
    # Add slash to name
    if name[0] != '/':
        name_without_slash = name
        name = '/' + name
    else:
        name_without_slash = name[1:]        

    # Update ros api
    ros.update()

    # Get service
    service = ros.get_service(name)

    # Handle post request
    if request.method == 'POST':
        print("Got service call request")
        req_msg = request.form['request']
        ret = {
            'success': True,
            'error_msg': None,
            'response': None
        }
        try:
            req = str_to_msg(req_msg, service.request_class)
            res = service.call(req)
            ret['response'] = str(res)
        except Exception as e:
            ret['success'] = False
            ret['error_msg'] = str(e)
        return jsonify(ret)

    # Check if a svg representation should be returned
    generate_svg = request.args.get('get', '', type=str) == 'svg'    

    if generate_svg is True:
        svg = service.svg()
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
        url = url_for('service.get_service_info', name=name_without_slash)
        img_data = url + '?get=svg'

        # Return rendered template
        return render_template('service.html', title=f'Service {name}',
                               active_menu_item='service',
                               url=url,
                               content=content,
                               msg_template=initialize_msg(service.request_class()),
                               **menu.get_items(active_item=url),
                               img_data=img_data)
                               #img_data=img_stream)
                               #img_data=urllib.parse.quote(img_stream.rstrip('\n')))