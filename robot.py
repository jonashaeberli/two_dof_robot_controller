from kinematics import kinematics
from hardware import hardware
from controller import controller
from sim import sim

class robot:
    def __init__(self, run_sim = False):
        self.sim = run_sim
        self.kinematics = kinematics()
        self.hardware = hardware()
        self.controller = controller()
        self.sim = sim()

        self.hardware.handover(self.kinematics, self.controller)
        self.controller.handover(self.hardware if run_sim is False else self.sim, False if run_sim is False else True, self.kinematics)

        self.initialized = True

    
    def is_initialized(self):
        return self.initialized