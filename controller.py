import numpy as np
import time

class controller:
    def __init__(self):
        self.publish_rate = 200 # Hz
        self.trajectory_resolution = 0.1 # mm
        self.dt = 1 / self.publish_rate

        self.max_linear_velocity = 800 # mm/s
        self.max_linear_acceleration = 2000 # mm/s^2 accel/deccel

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
        

    def check_zero_park(self):
        pass # TODO: We should move with limited acceleration and joint move to the end of normal joint range and check if the arm is at the correct location


    def moveL(self, x, y, start_x = None , start_y = None):
        if start_x is None and start_y is None:
            angles = self.trajectory[len(self.trajectory)-1]
            pos = self.kinematics.kinematics(kinematics_type="forward", upper_arm_angle=angles.get("upper_arm_angle"), lower_arm_angle=angles.get("lower_arm_angle"))
            start_x = pos.get("x")
            start_y = pos.get("y")
        elif start_x is None or start_y is None:
            print("Invalid Input received")
            return
        
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


    def calculate_max_velocity_and_acceleration(self):
        max_velocity = 0
        max_acceleration = 0
        prev_velocity_upper = prev_velocity_lower = 0

        points = self.trajectory
        time_difference = self.dt

        for i in range(1, len(points)):
            displacement_upper = points[i]['upper_arm_angle'] - points[i-1]['upper_arm_angle']
            displacement_lower = points[i]['lower_arm_angle'] - points[i-1]['lower_arm_angle']

            velocity_upper = displacement_upper / time_difference
            velocity_lower = displacement_lower / time_difference

            max_velocity = max(max_velocity, velocity_upper, velocity_lower)

            if i != 1:
                acceleration_upper = (velocity_upper - prev_velocity_upper) / time_difference
                acceleration_lower = (velocity_lower - prev_velocity_lower) / time_difference

                max_acceleration = max(max_acceleration, acceleration_upper, acceleration_lower)

            prev_velocity_upper = velocity_upper
            prev_velocity_lower = velocity_lower

        return {"max_velocity": max_velocity, "max_acceleration": max_acceleration}


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