import odrive
from odrive.enums import *
import numpy as np

class hardware:
    def __init__(self):
        self.upper_arm_serial = "326D34753233"
        self.upper_arm_reference_turns = -1.8 # in turns

        self.lower_arm_serial = "307234623030"
        self.lower_arm_reference_turns = 14.3 # in turns

        self.gear_ratio = 100 # Both arms have the same gear ration

        # Next we set the correct position for the Zero Position in Rotations
        self.upper_arm_zero_hardware_offset = np.deg2rad(112.50) # We work in rads from horizontal position
        self.upper_arm_zero = (self.upper_arm_zero_hardware_offset / (2 * np.pi) * self.gear_ratio) + self.upper_arm_reference_turns

        self.lower_arm_zero_hardware_offset = np.deg2rad(-12.5) # We work in rads from horizontal position
        self.lower_arm_zero = (self.lower_arm_zero_hardware_offset / (2 * np.pi) * self.gear_ratio) + self.lower_arm_reference_turns

        self.connected = False

        print("Upper Arm Zero: ", self.upper_arm_zero)
        print("Lower Arm Zero: ", self.lower_arm_zero)

    
    def handover(self, kinematics, controller):
        self.controller = controller
        self.kinematics = kinematics


    def setup(self):
        self.upper_arm_drive = odrive.find_any(serial_number=self.upper_arm_serial)
        self.lower_arm_drive = odrive.find_any(serial_number=self.lower_arm_serial)

        if self.upper_arm_drive is not None and self.lower_arm_drive is not None:
            self.connected = True
            print("Connected to ODrives")
            return True
        else:
            print("Could not connect to ODrives")
            return False

    
    def shutdown(self):
        if self.upper_arm_drive is not None:
            self.upper_arm_drive = None
        if self.lower_arm_drive is not None:
            self.lower_arm_drive = None


    def move(self, upper_arm_angle, lower_arm_angle):
        self.upper_arm_command_position = self.upper_arm_zero - upper_arm_angle / (2 * np.pi) * self.gear_ratio
        self.lower_arm_command_position = self.lower_arm_zero - (lower_arm_angle * -1) / (2 * np.pi) * self.gear_ratio

        if self.connected == False:
            print("Not connected to ODrives")
            return False

        self.upper_arm_drive.axis0.controller.input_pos = self.upper_arm_command_position
        self.lower_arm_drive.axis0.controller.input_pos = self.lower_arm_command_position
    

    def get_pos(self):
        pos = self.pos_to_angle(self.upper_arm_drive.axis0.encoder.pos_estimate, self.lower_arm_drive.axis0.encoder.pos_estimate)
        return {"upper_arm_angle": pos.get("upper_arm_angle"), "lower_arm_angle": pos.get("lower_arm_angle")}
    

    def pos_to_angle(self, upper_arm_angle, lower_arm_angle):
        return {"upper_arm_angle": (self.upper_arm_zero - self.upper_arm_command_position) * (2 * np.pi) / self.gear_ratio, "lower_arm_angle": -1 * ((self.lower_arm_zero - self.lower_arm_command_position) * (2 * np.pi) / self.gear_ratio)}