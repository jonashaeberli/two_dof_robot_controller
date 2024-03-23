import numpy as np
import time

class controller:
    def __init__(self):
        self.publish_rate = 200 # Hz
        self.trajectory_resolution = 0.1 # mm
        self.dt = 1 / self.publish_rate

        self.max_linear_velocity = 200 # mm/s
        self.max_linear_acceleration = 200 # mm/s^2 accel/deccel

        self.max_angular_velocity = np.pi # rad/s
        self.max_angular_acceleration = 12*np.pi # rad/s^2 accel/deccel

        self.trajectory = []
        self.trajectory_valid = False


    def handover(self, hardware, kinematics):
        self.hardware = hardware
        self.kinematics = kinematics


    def setup_controller(self):
        if self.hardware.setup() == True:
            print("Hardware/Sim setup successful")
            return True
        else:
            print("Hardware/Sim setup failed")
            return False
        

    def check_zero(self):
        pass # TODO: We should move with limited acceleration and joint move to the end of normal joint range and check if the arm is at the correct location


    def moveLconst(self, x, y, start_x, start_y): #TODO: We need to check the points in the trajectory to make sure they are valid (no collisions, within range, etc.) we have check_position and check_angles in kinematics.py
        distance = np.sqrt((x-start_x)**2 + (y-start_y)**2)
        time = distance / self.max_linear_velocity
        steps = int(time * self.publish_rate)
        x_traj = np.linspace(start_x, x, steps)
        y_traj = np.linspace(start_y, y, steps)
        for i in range(steps):
            pos = self.kinematics.kinematics(kinematics_type="inverse", x=x_traj[i], y=y_traj[i])
            self.trajectory.append(pos)


    def moveL(self, x, y, start_x , start_y):
        distance = np.sqrt((x - start_x)**2 + (y - start_y)**2) # Distance between start and end point

        # Set values for acceleration and deceleration as well as max velocity
        acceleration = self.max_linear_acceleration
        acceleration_distance = distance / 2
        max_linear_velocity = np.sqrt(2 * acceleration * acceleration_distance)

        # Adjust maximum velocity based on acceleration and deceleration distances
        if max_linear_velocity > self.max_linear_velocity:
            # Enough distance for both acceleration and deceleration
            print("Enough distance for both acceleration and deceleration")
            max_linear_velocity = self.max_linear_velocity
            acceleration_time = max_linear_velocity / acceleration
            acceleration_distance = 0.5 * acceleration * acceleration_time**2
            time_at_max_velocity = (distance - (2*acceleration_distance)) / max_linear_velocity
        else:
            # Not enough distance for both acceleration and deceleration, adjust max velocity
            print("Not enough distance for both acceleration and deceleration")
            acceleration_distance = distance / 2
            max_linear_velocity = np.sqrt(2 * acceleration * acceleration_distance)
            acceleration_time = max_linear_velocity / acceleration
            time_at_max_velocity = 0
        
        # Calculate total time
        total_time = (2*acceleration_time) + time_at_max_velocity
        
        # Calculate number of steps based on update rate
        steps = int(total_time * self.publish_rate * 10)
        
        # Generate time array for trajectory
        time_array = np.linspace(0, total_time, steps)
        iteration = 10
        
        # Generate trajectory points
        for t in time_array:
            iteration += 1
            if time_at_max_velocity == 0:
                if t <= acceleration_time:
                    displacement = 0.5 * acceleration * t**2
                else:
                    displacement = distance - 0.5 * acceleration * (t - total_time)**2
            else:
                if t <= acceleration_time:
                    displacement = 0.5 * acceleration * t**2
                elif t <= (acceleration_time + time_at_max_velocity):
                    displacement = acceleration_distance + max_linear_velocity * (t - acceleration_time)
                else:
                    t_deceleration = t - (acceleration_time + time_at_max_velocity)
                    displacement = acceleration_distance + max_linear_velocity * time_at_max_velocity + max_linear_velocity * t_deceleration - 0.5 * acceleration * t_deceleration**2
            
            if iteration % 10 == 0:
                pos = self.kinematics.kinematics(kinematics_type="inverse", x=start_x + (x-start_x) * (displacement / distance), y=start_y + (y - start_y) * (displacement / distance))
                self.trajectory.append(pos)
            elif iteration >= 10:
                iteration = 0

        # TODO: Execute trajectory if execution is enabled. Else we should return the trajectory


    def moveJconst(self, x, y, execution = True, start_x = None, start_y = None):
        if start_x is None and start_y is None:
            current_angle = self.hardware.get_pos()
            current_pos = self.kinematics.kinematics(kinematics_type="forward", upper_arm_angle=current_angle.get("upper_arm_angle"), lower_arm_angle=current_angle.get("lower_arm_angle"))
        elif start_x is not None and start_y is not None:
            current_pos = {"x": start_x, "y": start_y}
        else:
            print("Invalid Input received")
            return
        
        # First we need to find the travel distances for both joints. Therefore we need the inverse kineamtics to find start and end joint values
        goal_angles = self.kinematics.kinematics(kinematics_type="inverse", x=x, y=y)
        start_angles = self.kinematics.kinematics(kinematics_type="inverse", x=current_pos.get("x"), y=current_pos.get("y"))

        # Calculate the distance for each joint
        distance_upper = goal_angles.get("upper_arm_angle") - start_angles.get("lower_arm_angle")
        distance_lower = goal_angles.get("upper_arm_angle") - start_angles.get("lower_arm_angle")
        max_distance = distance_upper if distance_upper > distance_lower else distance_lower

        time = max_distance / self.max_angular_velocity
        steps = int(time * self.publish_rate)

        #TODO: Implement the trajectory generation for joint movements
    

    def validate_trajectory(self):
        for pos in self.trajectory:
            if self.kinematics.check_angles(pos.get("upper_arm_angle"), pos.get("lower_arm_angle")):
                continue
            else:
                return False
        self.trajectory_valid = True
        return True


    def execute_trajectory(self):
        debt_time = 0.0
        if self.trajectory_valid:
            for pos in self.trajectory:
                start_time = time.time()  # start timing
                self.hardware.move(pos.get("upper_arm_angle"), pos.get("lower_arm_angle"))
                end_time = time.time()  # end timing
                elapsed_time = end_time - start_time  # calculate elapsed time

                debt_time += elapsed_time - self.dt  # update debt time
                sleep_time = max(self.dt - elapsed_time, -debt_time)  # calculate sleep time, can't be more than debt time
                if sleep_time > 0:  # if there's time left, sleep
                    time.sleep(sleep_time)
                    debt_time += sleep_time - self.dt  # update debt time
        else:
            print("Trajectory is not valid or validated, can't execute!")
        return True


    def clear_trajectory(self):
        self.trajectory = []
        self.trajectory_valid = False