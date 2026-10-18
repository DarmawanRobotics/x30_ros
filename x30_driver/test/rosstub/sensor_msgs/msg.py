from rosstub_msg import Msg, header, quat, vec


class Imu(Msg):
    def __init__(self):
        self.header = header()
        self.orientation, self.angular_velocity, self.linear_acceleration = quat(), vec(), vec()


class JointState(Msg):
    def __init__(self):
        self.header, self.name = header(), []
        self.position, self.velocity, self.effort = [], [], []


class BatteryState(Msg):
    POWER_SUPPLY_STATUS_UNKNOWN = 0
    POWER_SUPPLY_HEALTH_UNKNOWN = 0
    POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0

    def __init__(self):
        self.header = header()
        self.voltage = self.current = self.charge = self.capacity = 0.0
        self.design_capacity = self.percentage = self.temperature = 0.0
        self.power_supply_status = self.power_supply_health = self.power_supply_technology = 0
        self.present = False
