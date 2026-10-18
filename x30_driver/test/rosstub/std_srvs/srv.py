class Trigger(object):
    pass


class TriggerResponse(object):
    def __init__(self, success=False, message=''):
        self.success, self.message = success, message

    def __repr__(self):
        return 'TriggerResponse(%r, %r)' % (self.success, self.message)
