import matplotlib.pyplot as plt
import numpy as np

from kinematics import kinematics

class sim:
    def __init__(self):
        self.upper_arm_length = 250  # Define the length of the arms
        self.lower_arm_length = 300
        self.upper_arm_angle = np.deg2rad(-5)
        self.lower_arm_angle = np.deg2rad(-100)

        self.kinematics = kinematics()

        # Create a new figure for the plot
        self.fig, self.ax = plt.subplots()

        # Create line objects for the arms
        self.upper_arm_line, = self.ax.plot([], [], 'r-')
        self.lower_arm_line, = self.ax.plot([], [], 'b-')

        # Set up the plot limits
        self.ax.set_xlim(-100, 500)
        self.ax.set_ylim(-100, 200)

        # Draw the plot for the first time
        self.update_sim()

        plt.show(block=False)

    def handover(self, kinematics):
        self.kinematics = kinematics

    def move(self, upper_arm_angle, lower_arm_angle):
        self.upper_arm_angle = upper_arm_angle
        self.lower_arm_angle = lower_arm_angle
        self.update_sim()

    def update_sim(self):
        # Calculate the positions of the arm joints
        upper_arm_x = self.upper_arm_length * np.cos(self.upper_arm_angle)
        upper_arm_y = self.upper_arm_length * np.sin(self.upper_arm_angle)

        lower_arm_x = upper_arm_x + self.lower_arm_length * np.cos(self.lower_arm_angle)
        lower_arm_y = upper_arm_y + self.lower_arm_length * np.sin(self.lower_arm_angle)

        # Update the positions of the line objects
        self.upper_arm_line.set_data([0, upper_arm_x], [0, upper_arm_y])
        self.lower_arm_line.set_data([upper_arm_x, lower_arm_x], [upper_arm_y, lower_arm_y])

        # Redraw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def get_pos(self):
        return {"upper_arm_angle": self.upper_arm_angle, "lower_arm_angle": self.lower_arm_angle}