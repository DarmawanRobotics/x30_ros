"""Attribute bags standing in for ROS messages (test_node.py)."""


class Msg(object):
    def __init__(self, **fields):
        for key, value in fields.items():
            setattr(self, key, value)


def vec():
    return Msg(x=0.0, y=0.0, z=0.0)


def quat():
    return Msg(x=0.0, y=0.0, z=0.0, w=1.0)


def header():
    return Msg(stamp=None, frame_id='')
