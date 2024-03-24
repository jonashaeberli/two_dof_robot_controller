import numpy as np
import time

class controller:
    def __init__(self):
        self.publish_rate = 100 # Hz
        self.trajectory_resolution = 0.1 # mm
        self.dt = 1 / self.publish_rate

        self.max_linear_velocity = 10 # mm/s
        self.max_linear_acceleration = 10 # mm/s^2 accel/deccel

        self.max_angular_velocity = np.pi / 40 # rad/s
        self.max_angular_acceleration = 10*np.pi # rad/s^2 accel/deccel

        self.trajectory = []
        self.trajectory_valid = False

        self.status = False


    def handover(self, hardware, sim, kinematics):
        self.hardware = hardware
        self.sim = sim
        self.kinematics = kinematics


    def setup_controller(self):
        status = self.hardware.setup()
        time.sleep(0.5)
        if status == True:
            print("Hardware/Sim setup successful")
            self.status = True
            return True
        else:
            print("Hardware/Sim setup failed")
            return False
        

    def check_zero_park(self):
        if self.status == True:
            input("Everything ready press Enter to go to zero position")
        else:
            print("Hardware/Sim not setup")
            return

        self.clear_trajectory()
        zero_position = self.kinematics.kinematics(kinematics_type="forward", upper_arm_angle=np.deg2rad(109.9), lower_arm_angle=np.deg2rad(9.9))
        self.moveJconst(zero_position.get("x"), zero_position.get("y"))
        if self.validate_trajectory():
            print("Trajectory is valid")
            input("Press Enter to go to reference position KEEP HANDS ON THE EMERGENCY STOP")
            self.execute_trajectory()
        input("Confirm that the robot is in the zero position and press Enter to continue to park position")
        self.clear_trajectory()
        park_position = self.kinematics.kinematics(kinematics_type="forward", upper_arm_angle=np.deg2rad(80), lower_arm_angle=np.deg2rad(-70))
        self.moveJconst(park_position.get("x"), park_position.get("y"), start_x=zero_position.get("x"), start_y=zero_position.get("y"))
        if self.validate_trajectory():
            print("Trajectory is valid")
            input("Press Enter to go to park position KEEP HANDS ON THE EMERGENCY STOP")
            self.execute_trajectory()
        self.clear_trajectory()


    def park(self):
        if self.status == True:
            input("Everything ready press Enter to go to zero position")
        else:
            print("Hardware/Sim not setup")
            return
        self.clear_trajectory()
        park_position = self.kinematics.kinematics(kinematics_type="forward", upper_arm_angle=np.deg2rad(80), lower_arm_angle=np.deg2rad(-70))
        self.moveL(park_position.get("x"), park_position.get("y"))
        if self.validate_trajectory():
            print("Trajectory is valid")
            input("Press Enter to go to park position KEEP HANDS ON THE EMERGENCY STOP")
            self.execute_trajectory()
        self.clear_trajectory()


    def moveL(self, x, y, start_x = None , start_y = None):
        if start_x is None and start_y is None:
            if len(self.trajectory) == 0: 
                pos = self.hardware.get_pos()
                start_x = pos.get("x")
                start_y = pos.get("y")
            else:
                index = len(self.trajectory) - 1
                while 'gripper' in self.trajectory[index] and index >= 0:
                    index -= 1
                if index >= 0:
                    angles = self.trajectory[index]
                    pos = self.kinematics.kinematics(kinematics_type="forward", upper_arm_angle=angles.get("upper_arm_angle"), lower_arm_angle=angles.get("lower_arm_angle"))
                    start_x = pos.get("x")
                    start_y = pos.get("y")
                else:
                    print("No valid angles found in trajectory")
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
        steps = int(total_time * self.publish_rate)
        
        # Generate time array for trajectory
        time_array = np.linspace(0, total_time, steps)
        
        # Generate trajectory points
        for t in time_array:
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
            

            pos = self.kinematics.kinematics(kinematics_type="inverse", x=start_x + (x-start_x) * (displacement / distance), y=start_y + (y - start_y) * (displacement / distance))
            self.trajectory.append(pos)


    def moveJconst(self, x, y, execution = True, start_x = None, start_y = None):
        if start_x is None and start_y is None:
            if len(self.trajectory) == 0: 
                pos = self.hardware.get_pos()
                start_x = pos.get("x")
                start_y = pos.get("y")
            else:
                index = len(self.trajectory) - 1
                while 'gripper' in self.trajectory[index] and index >= 0:
                    index -= 1
                if index >= 0:
                    angles = self.trajectory[index]
                    pos = self.kinematics.kinematics(kinematics_type="forward", upper_arm_angle=angles.get("upper_arm_angle"), lower_arm_angle=angles.get("lower_arm_angle"))
                    start_x = pos.get("x")
                    start_y = pos.get("y")
                else:
                    print("No valid angles found in trajectory")
        elif start_x is None or start_y is None:
            print("Invalid Input received")
            return
        
        # First we need to find the travel distances for both joints. Therefore we need the inverse kineamtics to find start and end joint values
        goal_angles = self.kinematics.kinematics(kinematics_type="inverse", x=x, y=y)
        start_angles = self.kinematics.kinematics(kinematics_type="inverse", x=start_x, y=start_y)

        # Calculate the distance for each joint
        distance_upper = goal_angles.get("upper_arm_angle") - start_angles.get("upper_arm_angle")
        distance_lower = goal_angles.get("lower_arm_angle") - start_angles.get("lower_arm_angle")
        max_distance = distance_upper if distance_upper > distance_lower else distance_lower

        time = np.absolute(max_distance / self.max_angular_velocity)
        steps = int(time * self.publish_rate)
        time_array = np.linspace(0, time, steps)

        for t in time_array:
            pos = {"upper_arm_angle": start_angles.get("upper_arm_angle") + distance_upper * (t / time), "lower_arm_angle": start_angles.get("lower_arm_angle") + distance_lower * (t / time)}
            self.trajectory.append(pos)


    def validate_trajectory(self):
        for pos in self.trajectory:
            if 'gripper' in pos:
                continue
            if not self.kinematics.check_angles(pos.get("upper_arm_angle"), pos.get("lower_arm_angle")):
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
            # Skip the point if it contains "gripper"
            if 'gripper' in points[i] or 'gripper' in points[i-1]:
                continue

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

        return {"max_velocity": (max_velocity/(2*np.pi))*self.hardware.gear_ratio, "max_acceleration": (max_acceleration/(2*np.pi))*self.hardware.gear_ratio}


    def execute_trajectory(self):
        debt_time = 0.0
        if self.trajectory_valid:
            for pos in self.trajectory:
                if "gripper" in pos and len(pos) == 1:
                    if pos.get("gripper") == "open" and self.sim is False:
                        self.hardware.gripper("open")
                    elif pos.get("gripper") == "close" and self.sim is False:
                        self.hardware.gripper("close")
                else:
                    start_time = time.time()  # start timing
                    self.hardware.move(pos.get("upper_arm_angle"), pos.get("lower_arm_angle"))
                    end_time = time.time()  # end timing

                    # Get the actual position
                    actual_pos = self.hardware.get_angles()

                    # Check if the actual position is within 2% of the commanded position
                    if abs(actual_pos['upper_arm_angle'] - pos.get('upper_arm_angle')) > 0.02 * pos.get('upper_arm_angle') or \
                       abs(actual_pos['lower_arm_angle'] - pos.get('lower_arm_angle')) > 0.02 * pos.get('lower_arm_angle'):
                        print("Error: Actual position is more than 0.5% out of the commanded position")
                        return
                    
                    elapsed_time = end_time - start_time  # calculate elapsed time

                    debt_time += elapsed_time - self.dt  # update debt time
                    sleep_time = max(self.dt - elapsed_time, -debt_time)  # calculate sleep time, can't be more than debt time
                    if sleep_time > 0:  # if there's time left, sleep
                        time.sleep(sleep_time)
                        debt_time += sleep_time - self.dt  # update debt time
            self.trajectory_valid = False
        else:
            print("Error: Trajectory is not valid")
            print("Trajectory is not valid or validated, can't execute!")
    

    def set_gripper(self, state):
        if state == "open":
            self.trajectory.append({"gripper": "open"})
        elif state == "close":
            self.trajectory.append({"gripper": "close"})
        else:
            print("Invalid state received")
            return False
        
    
    def pause_trajectory(self, time):
        steps = int(time * self.publish_rate)
        index = len(self.trajectory) - 1
        while 'gripper' in self.trajectory[index] and index >= 0:
            index -= 1
        if index >= 0:
            angles = self.trajectory[index]
            for i in range(steps):
                self.trajectory.append(angles)
        else:
            print("No valid angles found in trajectory")
    

    def set_move_params(self, linear_velocity, linear_acceleration):
        self.max_linear_velocity = linear_velocity
        self.max_linear_acceleration = linear_acceleration

    def clear_trajectory(self):
        self.trajectory = []
        self.trajectory_valid = False
