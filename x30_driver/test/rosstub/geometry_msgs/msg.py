from rosstub_msg import Msg, header, quat, vec


class Twist(Msg):
    def __init__(self):
        self.linear, self.angular = vec(), vec()


class TransformStamped(Msg):
    def __init__(self):
        self.header, self.child_frame_id = header(), ''
        self.transform = Msg(translation=vec(), rotation=quat())
