from rosstub_msg import Msg, header


class KeyValue(Msg):
    def __init__(self, key='', value=''):
        self.key, self.value = key, value


class DiagnosticStatus(Msg):
    OK, WARN, ERROR, STALE = 0, 1, 2, 3

    def __init__(self, level=0, name='', message='', hardware_id='', values=None):
        self.level, self.name, self.message, self.hardware_id = level, name, message, hardware_id
        self.values = values or []


class DiagnosticArray(Msg):
    def __init__(self):
        self.header, self.status = header(), []
