from rosstub_msg import Msg, header, quat, vec


class Odometry(Msg):
    def __init__(self):
        self.header, self.child_frame_id = header(), ''
        self.pose = Msg(pose=Msg(position=vec(), orientation=quat()), covariance=[0.0] * 36)
        self.twist = Msg(twist=Msg(linear=vec(), angular=vec()), covariance=(0.0,) * 36)
