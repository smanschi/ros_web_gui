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
        data = AttrDict(**data)
        for key in data:
            data[key] = _dict_to_msg(data[key])
        return data
    else:
        return data

def str_to_msg(data, cls):
    data_dict = yaml.safe_load(data)
    msg = _dict_to_msg(data_dict)
    return cls(**msg)