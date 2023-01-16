
class RWSRoutine():

    def __init__(self, l_routine=None, r_routine=None):
        self.l_routine = l_routine
        self.r_routine = r_routine

class EGMRoutine():

    def __init__(self, name, duration=0.0):
        self.name = name
        self.duration = duration


test_config = [EGMRoutine("testdemo"),
          RWSRoutine("rwstest0"),
          RWSRoutine("rwstest1")]

egm_only_config = [EGMRoutine("testdemo", 20)]

config = egm_only_config
    