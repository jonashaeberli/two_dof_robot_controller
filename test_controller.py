from robot import robot
import time

pnp = robot(run_sim = True)
if pnp.controller.setup_controller():
    print("Controller setup successful")
    pnp.controller.clear_trajectory()
    print(pnp.kinematics.kinematics(kinematics_type="forward", upper_arm_angle=1.1911444921527023, lower_arm_angle=-0.67832183387436))
    pnp.controller.moveL(500, 100, start_x=113.01, start_y=44.51)
    pnp.controller.moveL(500, 50)
    if pnp.controller.validate_trajectory():
        print("Trajectory is valid")
        pnp.controller.execute_trajectory()
    pnp.controller.clear_trajectory()
    pnp.controller.moveL(500, 100, start_x=500, start_y=50)
    pnp.controller.moveL(250, 100)
    pnp.controller.moveL(250, 50)
    pnp.controller.moveL(250, 100)
    pnp.controller.moveL(500, 100)
    pnp.controller.moveL(500, 50)
    print(pnp.controller.calculate_max_velocity_and_acceleration())
    if pnp.controller.validate_trajectory():
        print("Trajectory is valid")
        input("Press Enter to execute pnp")
        output = pnp.controller.execute_trajectory()