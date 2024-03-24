import matplotlib.pyplot as plt
import numpy as np

from kinematics import kinematics

class sim:
    def __init__(self):
        self.running = False

        self.gear_ratio = 100

    def handover(self, kinematics):
        self.kinematics = kinematics


    def setup(self):
        self.running = True
        self.upper_arm_angle = np.deg2rad(45)
        self.lower_arm_angle = np.deg2rad(-35)

        self.kinematics = kinematics()

        # Create a new figure for the plot
        self.fig, self.ax = plt.subplots()

        # Create line objects for the arms and levers
        self.upper_arm_line, = self.ax.plot([], [], 'r-')
        self.lower_arm_line, = self.ax.plot([], [], 'b-')
        self.lever_base, = self.ax.plot([], [], 'g-')
        self.lever_arm_extension, = self.ax.plot([], [], 'g-')
        self.lever_connection, = self.ax.plot([], [], 'g-')
        self.gripper, = self.ax.plot([], [], 'c-')
        # This point will be used to represent the end effektor position
        self.last_annotation = None

        self.move_counter = 0

        # Set up the plot limits
        self.ax.set_xlim(-600, 600)
        self.ax.set_ylim(-600, 600)

        self.ax.set_xticks(np.arange(-600, 600, 100))
        self.ax.set_yticks(np.arange(-600, 600, 100))

        # Show the grid
        self.ax.grid(True)

        # Draw the plot for the first time
        self.update_sim()

        plt.show(block=False)
        return True


    def move(self, upper_arm_angle, lower_arm_angle):
        self.upper_arm_angle = upper_arm_angle
        self.lower_arm_angle = lower_arm_angle

        self.move_counter += 1
        if self.move_counter % 10 == 0:
            self.update_sim()


    def update_sim(self):
        # Calculate the positions of the arm segments
        upper_arm_x = self.kinematics.upper_arm_length * np.cos(self.upper_arm_angle) + self.kinematics.x_rotation_offset
        upper_arm_y = self.kinematics.upper_arm_length * np.sin(self.upper_arm_angle) + self.kinematics.y_rotation_offset

        lower_arm_x = upper_arm_x + self.kinematics.lower_arm_length * np.cos(self.lower_arm_angle + self.kinematics.lever_arm_angle_offset)
        lower_arm_y = upper_arm_y + self.kinematics.lower_arm_length * np.sin(self.lower_arm_angle + self.kinematics.lever_arm_angle_offset)
        
        # Calculate the positions of the lever arm + connection
        lever_x = - (self.kinematics.lever_arm_length * np.cos(self.lower_arm_angle))
        lever_y = - (self.kinematics.lever_arm_length * np.sin(self.lower_arm_angle))

        lever_base_x = lever_x + self.kinematics.x_rotation_offset
        lever_base_y = lever_y + self.kinematics.y_rotation_offset

        lever_extension_x = upper_arm_x + lever_x
        lever_extension_y = upper_arm_y + lever_y

        endeffector_x = lower_arm_x + self.kinematics.gripper_x_offset
        endeffector_y = lower_arm_y + self.kinematics.gripper_y_offset

        # Update the positions of the line objects
        self.upper_arm_line.set_data([self.kinematics.x_rotation_offset, upper_arm_x], [self.kinematics.y_rotation_offset, upper_arm_y])
        self.lower_arm_line.set_data([upper_arm_x, lower_arm_x], [upper_arm_y, lower_arm_y])
        self.lever_base.set_data([self.kinematics.x_rotation_offset, lever_base_x], [self.kinematics.y_rotation_offset, lever_base_y])
        self.lever_arm_extension.set_data([upper_arm_x, lever_extension_x], [upper_arm_y, lever_extension_y])
        self.lever_connection.set_data([lever_base_x, lever_extension_x], [lever_base_y, lever_extension_y])
        self.gripper.set_data([lower_arm_x, endeffector_x], [lower_arm_y, endeffector_y])

        if self.last_annotation is not None:
            self.last_annotation.remove()
        self.last_annotation = self.ax.annotate(f'({endeffector_x:.2f}, {endeffector_y:.2f})', (endeffector_x, endeffector_y), textcoords="offset points", xytext=(0,10), ha='center')
        
        # Redraw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


    def get_pos(self):
        return self.kinematics.kinematics(kinematics_type="forward", upper_arm_angle=self.upper_arm_angle, lower_arm_angle=self.lower_arm_angle)