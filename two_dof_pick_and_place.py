import matplotlib.pyplot as plt
import numpy as np

class two_dof_pick_and_place:
    def __init__(self):
        """
        Initializes on Object of the two_dof_pick_and_place class.
        """
        self.upper_arm_length = 250
        self.lower_arm_length = 300
        self.lever_arm_length = 100
        self.lever_arm_angle_offset = np.deg2rad(-10)
        self.gripper_x_offset = 35
        self.gripper_y_offset = -16.25
        self.x_rotation_offset = -25
        self.y_rotation_offset = 110
        self.upper_arm_range = (np.deg2rad(110), np.deg2rad(-5))
        self.lower_arm_range = (np.deg2rad(10), np.deg2rad(-100))
        self.min_lever_distance = 52.5


    def kinematics(self, kinematics_type="forward", x=None, y=None, upper_arm_angle=None, lower_arm_angle=None):
        """
        Perform forward or inverse kinematics calculations for a 2-DOF robot arm.

        Parameters:
        - kinematics_type (str): The type of kinematics calculation to perform. Valid values are "forward" and "inverse".
        - x (float): The x-coordinate of the end effector position. Required for inverse kinematics.
        - y (float): The y-coordinate of the end effector position. Required for inverse kinematics.
        - upper_arm_angle (float): The angle of the upper arm in radians. Required for forward kinematics.
        - lower_arm_angle (float): The angle of the lower arm in radians. Required for forward kinematics.

        Returns:
        - If kinematics_type is "forward" and valid angles are provided, returns a dictionary with the calculated x and y coordinates of the end effector.
        - If kinematics_type is "inverse" and valid coordinates are provided, returns a dictionary with the calculated upper_arm_angle and lower_arm_angle in radians.
        - If the arguments are invalid or the kinematics_type is invalid, prints an error message and returns None.
        - If the position is out of the robot's reach, prints an error message and returns "out_of_reach".
        """
        if kinematics_type == "forward":
            if upper_arm_angle is not None and lower_arm_angle is not None:
                return { "x": (self.upper_arm_length * np.cos(upper_arm_angle) + self.lower_arm_length * np.cos(lower_arm_angle + self.lever_arm_angle_offset) + self.x_rotation_offset + self.gripper_x_offset),
                        "y": (self.upper_arm_length * np.sin(upper_arm_angle) + self.lower_arm_length * np.sin(lower_arm_angle + self.lever_arm_angle_offset) + self.y_rotation_offset + self.gripper_y_offset)}
            else:
                print("Invalid forward kinematics arguments")
                return    
        elif kinematics_type == "inverse":
            if x is not None and y is not None:
                x = x - self.gripper_x_offset - self.x_rotation_offset
                y = y - self.gripper_y_offset - self.y_rotation_offset
                c = np.sqrt(x**2 + y**2)
                upper_arm_angle = np.arccos((self.upper_arm_length**2 + c**2 - self.lower_arm_length**2) / (2 * self.upper_arm_length * c)) + np.arctan(y/x)
                lower_arm_angle = upper_arm_angle + np.arccos((self.lower_arm_length**2 + self.upper_arm_length**2 - c**2) / (2 * self.lower_arm_length * self.upper_arm_length)) - np.pi - self.lever_arm_angle_offset
                return { "upper_arm_angle": upper_arm_angle, "lower_arm_angle": lower_arm_angle} #in radians
            else:
                print("Invalid inverse kinematics arguments")
                return
        else:
            print("Invalid kinematics type")
            return
        
    def draw_mechanism_with_inverse_kinematic(self, x, y):
        # Calculate the inverse kinematic angles
        x = x - self.gripper_x_offset
        y = y - self.gripper_y_offset
        c = np.sqrt(x**2 + y**2)
        upper_arm_angle = np.arccos((self.upper_arm_length**2 + c**2 - self.lower_arm_length**2) / (2 * self.upper_arm_length * c)) + np.arctan(y/x)
        lower_arm_angle = upper_arm_angle + np.arccos((self.lower_arm_length**2 + self.upper_arm_length**2 - c**2) / (2 * self.lower_arm_length * self.upper_arm_length)) - np.pi

        return np.rad2deg(upper_arm_angle), np.rad2deg(lower_arm_angle)
    
    def check_position(self, x, y):
        """
        Check if a given position is within the robot's reach.

        Parameters:
        - x (float): The x-coordinate of the end effector position.
        - y (float): The y-coordinate of the end effector position.

        Returns:
        - True if the position is within the robot's reach, False otherwise.
        """
        print(f"Checking position: {x}|{y}")
        angles = self.kinematics(kinematics_type="inverse", x=x, y=y) != "out_of_reach"
        if angles != "out_of_reach" and angles is not None:
            if x >= 51.25: # Check Interfearance with Base mount
                if self.upper_arm_range[0] >= angles.get("upper_arm_angle") >= self.upper_arm_range[1] and self.lower_arm_range[0] >= angles.get("lower_arm_angle") >= self.lower_arm_range[1]: # Check if angles are within allowable range
                    print("Position is within allowable angle range")
                    

                    return True


    def lever_distance (self, upper_arm_angle, lower_arm_angle):
        return (np.sin(np.pi - upper_arm_angle - lower_arm_angle) * self.lever_arm_length)


    def draw_required_work_envelope(self, x_por_offset, y_por_offset):
        # Define the coordinates of the work envelope points
        x = [0.16+x_por_offset, 0.16+x_por_offset, 0.5+x_por_offset, 0.32+x_por_offset, 0.16+x_por_offset,]
        y = [0.06-y_por_offset, -0.0375-y_por_offset, -0.0375-y_por_offset, 0.06-y_por_offset, 0.06-y_por_offset,]

        # Plot the work envelope
        self.ax.fill_between(x, y, color='skyblue', alpha=0.5)