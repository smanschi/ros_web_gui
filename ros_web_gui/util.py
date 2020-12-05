import roslib
import rospy
import sys
import yaml

def get_object_size(obj, seen=None):
    """Recursively finds size of objects
    Copied from https://goshippo.com/blog/measure-real-size-any-python-object/
    """
    size = sys.getsizeof(obj)
    
    if seen is None:
        seen = set()
    obj_id = id(obj)
    if obj_id in seen:
        return 0
    # Important mark as seen *before* entering recursion to gracefully handle
    # self-referential objects
    seen.add(obj_id)
    if isinstance(obj, dict):
        size += sum([get_object_size(v, seen) for v in obj.values()])
        size += sum([get_object_size(k, seen) for k in obj.keys()])

    if hasattr(obj, '__dict__'):
        size += get_object_size(obj.__dict__, seen)

    if hasattr(obj, '__slots__'):
        size += get_object_size(obj.__slots__, seen)

    if hasattr(obj, '__iter__') and not isinstance(obj, (str, bytes, bytearray)):
        size += sum([get_object_size(i, seen) for i in obj])
    return size

class AttrDict(dict):
    def __init__(self, *args, **kwargs):
        super(AttrDict, self).__init__(*args, **kwargs)
        self.__dict__ = self

def _dict_to_msg(data):
    if isinstance(data, dict):    
        # ToDo: Not really a nice special case for timestamps
        if 'secs' in data and 'nsecs' in data:
            return rospy.get_rostime()
        data = AttrDict(**data)
        for key in data:
            data[key] = _dict_to_msg(data[key])
        return data
    elif isinstance(data, list):
        for idx, ele in enumerate(data):
            data[idx] = _dict_to_msg(ele)
        return data
    else:
        return data

def str_to_msg(data, cls):
    data_dict = yaml.safe_load(data)
    if data_dict is None or isinstance(data_dict, str):
        return cls()
    msg = _dict_to_msg(data_dict)
    obj = cls(**msg)
    return obj

def initialize_msg(msg):
    primitives = {
        "bool": False,
        "int8": 0,
        "uint8": 0,
        "int16": 0,
        "uint16": 0,
        "int32": 0,
        "uint32": 0,
        "int64": 0,
        "uint64": 0,
        "float32": 0.0,
        "float64": 0.0,
        "string": "",
        #"time": rospy.Time.now(),
        #"duration": rospy.Duration()
    }
    
    for idx, name in enumerate(msg.__slots__):
        msg_type = msg._slot_types[idx]
        if msg_type.endswith('[]'):
            msg_type = msg_type[:-2]
            if msg_type in primitives:
                setattr(msg, name, [primitives[msg_type]])
            else:
                msg_class = roslib.message.get_message_class(msg_type[:-2])
                setattr(msg, name, [initialize_msg(msg_class())])
        elif msg_type in primitives:
            setattr(msg, name, primitives[msg_type])
        else:
            msg_class = roslib.message.get_message_class(msg_type)
            setattr(msg, name, initialize_msg(msg_class()))
    return msg