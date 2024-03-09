import matplotlib.pyplot as plt
import numpy as np

class two_dof_pick_and_place:
    def __init__(self, upper_arm_length, lower_arm_length, lever_arm_length, lever_arm_angle_offset, gripper_x_offset, gripper_y_offset):
        """
        Initializes the TwoDOFPickAndPlace class.

        Args:
            upper_arm_length (float): Length of the upper arm in millimeters.
            lower_arm_length (float): Length of the lower arm in millimeters.
            lever_arm_length (float): Length of the lever arm in millimeters.
            lever_arm_angle_offset (float): Angle offset of the lever arm in radians.
            gripper_x_offset (float): X offset of the gripper in millimeters.
            gripper_y_offset (float): Y offset of the gripper in millimeters.
        """
        self.lower_arm_length = lower_arm_length
        self.upper_arm_length = upper_arm_length
        self.lever_arm_length = lever_arm_length
        self.lever_arm_angle_offset = lever_arm_angle_offset
        self.gripper_x_offset = gripper_x_offset
        self.gripper_y_offset = gripper_y_offset
        self.fig, self.ax = plt.subplots()


    def start_plot(self):
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-0.6, 0.6)
        self.ax.set_ylim(-0.6,0.6)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        plt.grid(True)


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
                return { "x": self.upper_arm_length * np.cos(upper_arm_angle) + self.lower_arm_length * np.cos(lower_arm_angle) + self.gripper_x_offset,
                        "y": self.upper_arm_length * np.sin(upper_arm_angle) + self.lower_arm_length * np.sin(lower_arm_angle) + self.gripper_y_offset}
            else:
                print("Invalid forward kinematics arguments")
                return    
        elif kinematics_type == "inverse":
            if x is not None and y is not None:
                x = x - self.gripper_x_offset
                y = y - self.gripper_y_offset
                c = np.sqrt(x**2 + y**2)
                if c > self.upper_arm_length + self.lower_arm_length or c < self.upper_arm_length - self.lower_arm_length:
                    print("Position out of robot's reach!")
                    return "out_of_reach"
                upper_arm_angle = np.arccos((self.upper_arm_length**2 + c**2 - self.lower_arm_length**2) / (2 * self.upper_arm_length * c)) + np.arctan(y/x)
                lower_arm_angle = upper_arm_angle + np.arccos((self.lower_arm_length**2 + self.upper_arm_length**2 - c**2) / (2 * self.lower_arm_length * self.upper_arm_length)) - np.pi
                return { "upper_arm_angle": upper_arm_angle, "lower_arm_angle": lower_arm_angle} #in radians
            else:
                print("Invalid inverse kinematics arguments")
                return
        else:
            print("Invalid kinematics type")
            return
        
    
    def check_position(self, x, y):
        print("Checking position")
    
    
    def remove_ploted_mechanism(self, ploted_lines):
        for line in ploted_lines:
            line.remove()
        self.ax.clear()
        self.fig.canvas.draw()


    def draw_required_work_envelope(self, x_por_offset, y_por_offset):
        # Define the coordinates of the work envelope points
        x = [0.16+x_por_offset, 0.16+x_por_offset, 0.5+x_por_offset, 0.32+x_por_offset, 0.16+x_por_offset,]
        y = [0.06-y_por_offset, -0.0375-y_por_offset, -0.0375-y_por_offset, 0.06-y_por_offset, 0.06-y_por_offset,]

        # Plot the work envelope
        self.ax.fill_between(x, y, color='skyblue', alpha=0.5)


    def show_plot(self):
        plt.show(block=False)
        plt.pause(0.001)