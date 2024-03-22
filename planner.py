import numpy as np
from hardware import hardware
from kinematics import kinematics

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

        self.hardware = hardware()
        self.kinematics = kinematics()


    def moveLconst(self, x, y): #TODO: We need to check the points in the trajectory to make sure they are valid (no collisions, within range, etc.) we have check_position and check_angles in kinematics.py
        current_angle = self.hardware.get_pos()
        current_pos = self.kinematics.kinematics(kinematics_type="forward", upper_arm_angle=current_angle.get("upper_arm_angle"), lower_arm_angle=current_angle.get("lower_arm_angle"))
        distance = np.sqrt((x-current_pos.get("x"))**2 + (y-current_pos.get("y"))**2)
        time = distance / self.max_linear_velocity
        steps = int(time * self.publish_rate)
        x_traj = np.linspace(current_pos.get("x"), x, steps)
        y_traj = np.linspace(current_pos.get("y"), y, steps)
        for i in range(steps):
            pos = self.kinematics.kinematics(kinematics_type="inverse", x=x_traj[i], y=y_traj[i])
            self.trajectory.append(pos)

    
    def moveL(self, x, y):
        import numpy as np

def move_with_acceleration(self, x, y):
    current_angle = self.hardware.get_pos()
    current_pos = self.kinematics.kinematics(kinematics_type="forward", upper_arm_angle=current_angle.get("upper_arm_angle"), lower_arm_angle=current_angle.get("lower_arm_angle"))
    
    distance = np.sqrt((x - current_pos.get("x"))**2 + (y - current_pos.get("y"))**2)
    max_linear_velocity = self.max_linear_velocity
    acceleration = self.max_linear_acceleration
    deceleration = self.max_linear_acceleration # We do it like that that we later can have different acceleration and deceleration values
    
    acceleration_time = max_linear_velocity / acceleration
    deceleration_time = max_linear_velocity / deceleration # We do it like that that we later can have different acceleration and deceleration values
    
    acceleration_distance = 0.5 * acceleration * acceleration_time**2
    deceleration_distance = 0.5 * deceleration * deceleration_time**2 # We do it like that that we later can have different acceleration and deceleration values
    
    # Adjust maximum velocity based on acceleration and deceleration distances
    if (acceleration_distance + deceleration_distance) > distance:
        # Enough distance for both acceleration and deceleration
        time_at_max_velocity = (distance - acceleration_distance - deceleration_distance) / max_linear_velocity
    else:
        # Not enough distance for both acceleration and deceleration, adjust max velocity
        max_linear_velocity = np.sqrt(distance * (acceleration + deceleration))
        time_at_max_velocity = 0
    
    # Calculate total time
    total_time = acceleration_time + time_at_max_velocity + deceleration_time
    
    # Calculate number of steps based on update rate
    steps = int(total_time * self.publish_rate)
    
    # Generate time array for trajectory
    time_array = np.linspace(0, total_time, steps)
    
    # Generate trajectory points
    for t in time_array:
        if t <= acceleration_time:
            # Acceleration phase
            v = acceleration * t
        elif t <= acceleration_time + time_at_max_velocity:
            # Constant velocity phase
            v = max_linear_velocity
        else:
            # Deceleration phase
            v = max_linear_velocity - deceleration * (t - acceleration_time - time_at_max_velocity)
        
        # Calculate position at time t
        displacement = 0.5 * (acceleration * t**2) if t <= acceleration_time else acceleration_distance
        displacement += (max_linear_velocity * (t - acceleration_time)) if t <= acceleration_time + time_at_max_velocity else acceleration_distance + max_linear_velocity * (t - acceleration_time - time_at_max_velocity)
        displacement += (max_linear_velocity * (deceleration_time - (t - acceleration_time - time_at_max_velocity)) - 0.5 * deceleration * (deceleration_time - (t - acceleration_time - time_at_max_velocity))**2) if t > total_time - deceleration_time else distance

        pos = self.kinematics.kinematics(kinematics_type="inverse", x=current_pos.get("x") + (x - current_pos.get("x")) * (displacement / distance), y=current_pos.get("y") + (y - current_pos.get("y")) * (displacement / distance))
        self.trajectory.append(pos)


    def clear_trajectory(self):
        self.trajectory = []