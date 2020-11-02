import rosgraph
import rosnode
import rostopic
import socket

def get_info():
    # get master
    master = rosgraph.Master('rosnode')

    # go through the master system state first
    try:
        state = master.getSystemState()
    except socket.error:
        raise rosnode.ROSNodeIOException("Unable to communicate with master!")

    return {
        'pubs': [{'topic': s[0], 'publisher':  s[1], 'type': rostopic.get_topic_type(s[0])[0]} for s in state[0]],
        'subs': [{'topic': s[0], 'subscriber': s[1], 'type': rostopic.get_topic_type(s[0])[0]} for s in state[1]],
        'srvs': [{'topic': s[0], 'provider':   s[1], 'type': rostopic.get_topic_type(s[0])[0]} for s in state[2]],
    }