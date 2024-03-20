import odrive
from odrive.enums import *
from kinematics import kinematics
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

        self.kinematics = kinematics()

        self.connected = False

        print("Upper Arm Zero: ", self.upper_arm_zero)
        print("Lower Arm Zero: ", self.lower_arm_zero)


    def setup(self):
        self.upper_arm_drive = odrive.find_any(serial_number=self.upper_arm_serial)
        self.lower_arm_drive = odrive.find_any(serial_number=self.lower_arm_serial)

        if self.upper_arm_drive is not None and self.lower_arm_drive is not None:
            self.connected = True
            print("Connected to ODrives")
        else:
            print("Could not connect to ODrives")


    def move_to(self, upper_arm_angle, lower_arm_angle):
        self.upper_arm_command_position = self.upper_arm_zero - upper_arm_angle / (2 * np.pi) * self.gear_ratio
        self.lower_arm_command_position = self.lower_arm_zero - (lower_arm_angle * -1) / (2 * np.pi) * self.gear_ratio

        if self.connected == False:
            print("Not connected to ODrives")
            return False

        if self.kinematics.check_angles(upper_arm_angle, lower_arm_angle):
            self.upper_arm_drive.axis0.controller.input_pos = self.upper_arm_command_position
            self.lower_arm_drive.axis0.controller.input_pos = self.lower_arm_command_position
        else:
            print("Invalid Position or collision detected. This should not happen! There has to be an error in the planner")
            return False
        
        return True
