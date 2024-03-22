from robot import robot
import time

pnp = robot(run_sim = True)
if pnp.controller.setup_controller():
    print("Controller setup successful")
    pnp.controller.clear_trajectory()
    print(pnp.kinematics.kinematics(kinematics_type="forward", upper_arm_angle=1.1911444921527023, lower_arm_angle=-0.67832183387436))
    pnp.controller.moveL(350, 100, 350, 150)
    pnp.controller.moveL(150, 100, 350, 100)
    pnp.controller.moveL(350, 100, 150, 100)
    if pnp.controller.validate_trajectory():
        print("Trajectory is valid")
        pnp.controller.execute_trajectory()