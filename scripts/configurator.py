
class RWSRoutine():

    def __init__(self, l_routine=None, r_routine=None):
        self.l_routine = l_routine
        self.r_routine = r_routine

class EGMRoutine():

    def __init__(self, name, duration=None):
        self.name = name
        self.duration = duration


config = [RWSRoutine("dadayeh0"),
          RWSRoutine("dadayeh1")]

config = [EGMRoutine("testdemo")]
    