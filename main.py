from two_dof_plotter import MechanismPlotter
import time

# Create an object of MechanismPlotter
two_dof_robot_plot = MechanismPlotter(0.25, 0.3, 0.1, 0, 0)
two_dof_robot_plot.start_plot()
two_dof_robot_plot.draw_required_work_envelope(0, 0)
two_dof_robot_plot.draw_mechanism_with_inverse_kinematic(0.5, -0.0375)
two_dof_robot_plot.show_plot()
time.sleep(0.1)

for i in range(500, 160, -5):
    two_dof_robot_plot.draw_mechanism_with_inverse_kinematic(i/1000, -0.0375)
    time.sleep(0.05)
