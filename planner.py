import numpy as np

class planner:
    def __init__(self):
        self.publish_rate = 200 # Hz
        self.trajectory_resolution = 0.1 # mm
        self.dt = 1 / self.publish_rate

        self.max_linear_velocity = 100 # mm/s
        self.max_linear_acceleration = 10 # mm/s^2 accel/deccel

        self.max_angular_velocity = 6 # turns/s
        self.max_angular_acceleration = 6 # turns/s^2 accel/deccel

        self.trajectory = []
    
    def moveLconst(self, x, y):
        