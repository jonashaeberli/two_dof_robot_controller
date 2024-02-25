import matplotlib.pyplot as plt
import numpy as np

class MechanismPlotter:
    def __init__(self, upper_arm_length, lower_arm_length, lever_arm_length, gripper_x_offset, gripper_y_offset):
        self.lower_arm_length = lower_arm_length
        self.upper_arm_length = upper_arm_length
        self.lever_arm_length = lever_arm_length
        self.gripper_x_offset = gripper_x_offset
        self.gripper_y_offset = gripper_y_offset
        self.ploted_lines = []
        self.fig, self.ax = plt.subplots()

    def start_plot(self):
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-0.6, 0.6)
        self.ax.set_ylim(-0.6,0.6)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        plt.grid(True)

    def draw_mechanism_with_forward_kinematic(self, upper_arm_angle, lower_arm_angle):
        # Calculate the coordinates of the lower and upper arm endpoints
        upper_arm_x = self.upper_arm_length * np.cos(upper_arm_angle)
        upper_arm_y = self.upper_arm_length * np.sin(upper_arm_angle)
        lower_arm_x = upper_arm_x + self.lower_arm_length * np.cos(lower_arm_angle)
        lower_arm_y = upper_arm_y + self.lower_arm_length * np.sin(lower_arm_angle)
        lever_arm_x = -(self.lever_arm_length * np.cos(lower_arm_angle))
        lever_arm_y = -(self.lever_arm_length * np.sin(lower_arm_angle))


        # Plot the lines
        if len(self.ploted_lines) > 0:
            self.ploted_lines[0].set_data([0, upper_arm_x], [0, upper_arm_y])
            self.ploted_lines[1].set_data([upper_arm_x, lower_arm_x], [upper_arm_y, lower_arm_y])
            self.ploted_lines[2].set_data([0, lever_arm_x], [0, lever_arm_y])
            self.ploted_lines[3].set_data([upper_arm_x, upper_arm_x + lever_arm_x], [upper_arm_y, upper_arm_y + lever_arm_y])
            self.ploted_lines[4].set_data([lever_arm_x, upper_arm_x + lever_arm_x], [lever_arm_y, upper_arm_y + lever_arm_y])
            # Draw the updated plot
            self.fig.canvas.draw()
            plt.pause(0.001)
        else:
            # Plot the lines
            line1, = self.ax.plot([0, upper_arm_x], [0, upper_arm_y], 'b-', linewidth=2)
            line2, = self.ax.plot([upper_arm_x, lower_arm_x], [upper_arm_y, lower_arm_y], 'r-', linewidth=2)
            line3, = self.ax.plot([0, lever_arm_x], [0, lever_arm_y], 'g-', linewidth=1)
            line4, = self.ax.plot([upper_arm_x, upper_arm_x + lever_arm_x], [upper_arm_y, upper_arm_y + lever_arm_y], 'g-', linewidth=1)
            line5, = self.ax.plot([lever_arm_x, upper_arm_x + lever_arm_x], [lever_arm_y, upper_arm_y + lever_arm_y], 'g-', linewidth=1)

            # Add the lines to the list of lines to remove
            self.ploted_lines.extend([line1, line2, line3, line4, line5])

        # Calculate the distance between the upper arm and the mechanism for the lower arm
        distance = np.sin(np.pi-(-lower_arm_angle)-upper_arm_angle) * self.lever_arm_length
        x = lower_arm_x + self.gripper_x_offset
        y = lower_arm_y + self.gripper_y_offset

        return x, y, distance


    def draw_mechanism_with_inverse_kinematic(self, x, y):
        # Calculate the inverse kinematic angles
        x = x - self.gripper_x_offset
        y = y - self.gripper_y_offset
        c = np.sqrt(x**2 + y**2)
        upper_arm_angle = np.arccos((self.upper_arm_length**2 + c**2 - self.lower_arm_length**2) / (2 * self.upper_arm_length * c)) + np.arctan(y/x)
        lower_arm_angle = upper_arm_angle + np.arccos((self.lower_arm_length**2 + self.upper_arm_length**2 - c**2) / (2 * self.lower_arm_length * self.upper_arm_length)) - np.pi

        # Draw the mechanism
        ploted_lines = self.draw_mechanism_with_forward_kinematic(upper_arm_angle, lower_arm_angle)

        return upper_arm_angle, lower_arm_angle
    
    
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