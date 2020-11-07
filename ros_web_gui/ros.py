import rosgraph
import rosnode
import rosparam
import rostopic
import socket
import sys

def get_info():
    # get master
    master = rosgraph.Master('rosnode')

    # go through the master system state first
    try:
        state = master.getSystemState()
    except socket.error:
        raise rosnode.ROSNodeIOException("Unable to communicate with master!")
    
    param_names = []
    try:
        param_names = rosparam.list_params('')
    except rosparam.RosParamIOException:
        print('Could not fetch parameter names from server', file=sys.stdout)
    param_names = sorted(set(['/'.join(param.split('/')[:2]) for param in param_names]))

    return {
        'pubs': [{'topic': s[0], 'publisher':  s[1], 'type': rostopic.get_topic_type(s[0])[0]} for s in state[0]],
        'subs': [{'topic': s[0], 'subscriber': s[1], 'type': rostopic.get_topic_type(s[0])[0]} for s in state[1]],
        'srvs': [{'topic': s[0], 'provider':   s[1], 'type': rostopic.get_topic_type(s[0])[0]} for s in state[2]],
        'params': [{'name': s, 'type': 'unknown'} for s in param_names]
    }