import matplotlib.pyplot as plt
import numpy as np
import warnings
import ezdxf
from tqdm import tqdm
from scipy.spatial import ConvexHull


class two_dof_pick_and_place:
    def __init__(self):
        self.upper_arm_length = 250
        self.lower_arm_length = 300
        self.lever_arm_length = 115
        self.lever_arm_angle_offset = np.deg2rad(-10)
        self.gripper_x_offset = 35
        self.gripper_y_offset = -16.25
        self.x_rotation_offset = -25
        self.y_rotation_offset = 110
        self.upper_arm_range = (np.deg2rad(110), np.deg2rad(-5))
        self.lower_arm_range = (np.deg2rad(10), np.deg2rad(-100))
        self.min_lever_distance = 52.5

        # Work envelope parameters + calculation resolution
        self.min_x = 50
        self.max_x = 600
        self.min_y = -400
        self.max_y = 400
        self.resolution = 50

        warnings.filterwarnings("error", category=RuntimeWarning)


    def kinematics(self, kinematics_type="forward", x=None, y=None, upper_arm_angle=None, lower_arm_angle=None):
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
        
    
    def check_position(self, x, y):
        try:
            angles = self.kinematics(kinematics_type="inverse", x=x, y=y)
            if x >= 90: # Check Interfearance with Base mount
                if self.upper_arm_range[0] >= angles.get("upper_arm_angle") >= self.upper_arm_range[1] and self.lower_arm_range[0] >= angles.get("lower_arm_angle") >= self.lower_arm_range[1]: # Check if angles are within allowable range
                    if self.lever_distance(angles.get("upper_arm_angle"), angles.get("lower_arm_angle")) >= self.min_lever_distance: # Check if lever distance is greater than minimum
                        return True
        except RuntimeWarning as rw:
            pass
        return False


    def lever_distance (self, upper_arm_angle, lower_arm_angle):
        return np.sin(np.pi-(-lower_arm_angle)-upper_arm_angle) * self.lever_arm_length


    def check_work_envelope(self):
        # Check points in possible work envelope
        points = []
        total_iterations = ((self.max_x - self.min_x) // (self.resolution/100)) * ((self.max_y - self.min_y) // (self.resolution/100))
        progress_bar = tqdm(total=total_iterations, desc='Progress', unit=' iteration')

        for x in range(self.min_x*100, self.max_x*100, self.resolution):
            x = x / 100
            for y in range(self.min_y*100, self.max_y*100, self.resolution):
                y = y / 100
                if self.check_position(x, y):
                    points.append((x, y))
                progress_bar.update(1)
        progress_bar.close()

        # Get max and min points for each x
        y_values = {}
        for x, y in points:
            if x in y_values:
                # Update the highest y value
                y_values[x]['max'] = max(y_values[x]['max'], y)
                # Update the lowest y value
                y_values[x]['min'] = min(y_values[x]['min'], y)
            else:
                # Initialize the dictionary for this x
                y_values[x] = {'max': y, 'min': y}
        max_points = [(x, y_values[x]['max']) for x in y_values]
        min_points = [(x, y_values[x]['min']) for x in y_values]

        # Create the dxf file
        doc = ezdxf.new()
        msp = doc.modelspace()
        for i in range(len(max_points) - 1):
            msp.add_line(max_points[i], max_points[i + 1])
        for i in range(len(min_points) - 1):
            msp.add_line(min_points[i], min_points[i + 1])
        msp.add_line(max_points[0], min_points[0])
        msp.add_line(max_points[-1], min_points[-1])
        doc.saveas("plot.dxf")
        

    def draw_required_work_envelope(self):
        x = [160, 160, 500, 320, 160]
        y = [60, -37.5, -37.5, 60, 60]

        # Plot the work envelope
        self.axs[0].fill_between(x, y, color='skyblue', alpha=0.5)