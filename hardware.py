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
        self.upper_arm_zero_hardware_offset = 112.50 # We work in degrees from horizontal position
        self.upper_arm_zero = (self.upper_arm_zero_hardware_offset / 360 * self.gear_ratio) + self.upper_arm_reference_turns

        self.lower_arm_zero_hardware_offset = -12.5 # We work in degrees from horizontal position
        self.lower_arm_zero = (self.lower_arm_zero_hardware_offset / 360 * self.gear_ratio) + self.lower_arm_reference_turns

        self.kinematics = kinematics()

        print("Upper Arm Zero: ", self.upper_arm_zero)
        print("Lower Arm Zero: ", self.lower_arm_zero)


    def setup(self):
        self.upper_arm_drive = odrive.find_any(serial_number=self.upper_arm_serial)
        self.lower_arm_drive = odrive.find_any(serial_number=self.lower_arm_serial)

        # Do a check to see if the drives connected successfully
        print("Upper Arm Errors: ", odrive.dump_errors(self.upper_arm_drive))
        print("")
        print("Lower Arm Errors: ", odrive.dump_errors(self.lower_arm_drive))

        # Ask User if there are any errors
        input("Press Enter to continue if there are no errors...")


    def move_to(self, upper_arm_angle, lower_arm_angle):
        self.upper_arm_command_position = self.upper_arm_zero - upper_arm_angle/360*self.gear_ratio
        self.lower_arm_command_position = self.lower_arm_zero - lower_arm_angle/360*self.gear_ratio

        pos = self.kinematics.kinematics(kinematics_type="forward", upper_arm_angle=np.deg2rad(upper_arm_angle), lower_arm_angle=np.deg2rad(lower_arm_angle))

        if self.kinematics.check_position(pos.get("x"), pos.get("y")):
            self.upper_arm_drive.axis0.controller.input_pos = self.upper_arm_command_position
            self.lower_arm_drive.axis0.controller.input_pos = self.lower_arm_command_position
        else:
            print("Invalid Position or collision detected. This should not happen! There has to be an error in the planner")


    def move_dummy(self, upper_arm_angle, lower_arm_angle):
        self.upper_arm_command_position = self.upper_arm_zero - upper_arm_angle/360*self.gear_ratio
        self.lower_arm_command_position = self.lower_arm_zero - lower_arm_angle/360*self.gear_ratio

        print("Moving to: ", self.upper_arm_command_position, "and", self.lower_arm_command_position)