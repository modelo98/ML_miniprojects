import numpy as np
import matplotlib.pyplot as plt
from src import math_utils as mu

# WITH P-CONTROLLER

robot_position = np.array([2.0,1.0])
goal_position = np.array([7.0,5.0])
robot_theta = -45.0 
v = 0.5
dt = 0.01
num_steps = 5000
k_angular = 0.5
trajectory = []
thetas = []
trajectory.append(robot_position.copy())
thetas.append(robot_theta)
approximation = 0.01

for i in range(num_steps):

    heading = mu.heading_from_angle(robot_theta)
    robot_position[0] = robot_position[0] + v * heading[0] * dt
    robot_position[1] = robot_position[1] + v * heading[1] * dt
    vector_to_goal = goal_position - robot_position
    #target_angle = mu.angle_of_vector(goal_position)
    target_angle = np.rad2deg(mu.angle_of_vector(vector_to_goal))
    angle_error = target_angle - robot_theta
    angle_error = mu.normalize_angle_deg(angle_error)
    omega = k_angular * angle_error
    robot_theta += omega * dt

    if i == 0: minimum_angle_error = angle_error
    if angle_error < minimum_angle_error:
        minimum_angle_error = angle_error

    trajectory.append(robot_position.copy())
    thetas.append(robot_theta)

    print("-Step", i)
    print("Robot position: {} {}".format(np.round(robot_position[0],5),np.round(robot_position[1],5)))
    print("New theta:", robot_theta)
    print("Angle error:", angle_error)
    print("Minimum, angle error:", minimum_angle_error)
    print("\n")

    if (goal_position[0] - approximation) < robot_position[0] < (goal_position[0] + approximation) and (goal_position[1] - approximation) < robot_position[1] < (goal_position[1] + approximation):
        break

trajectory = np.array(trajectory)
initial_distance = mu.euclidean_distance(trajectory[0],goal_position)
final_distance = mu.euclidean_distance(trajectory[-1],goal_position)
print("Distance start to goal:", initial_distance)
print("Distance final step to goal", final_distance)
print("Last angle error:", angle_error)
print("Minimum angle error:", minimum_angle_error)

plt.scatter(trajectory[:, 0], trajectory[:, 1], label="Trajectory points")
plt.scatter(trajectory[0][0], trajectory[0][1], label="Initial position")
plt.scatter(trajectory[-1][0], trajectory[-1][1], label="Final position")
plt.plot(
    trajectory[:, 0],
    trajectory[:, 1],
    label="Trajectory line"
)

for i in range(0, len(trajectory) - 1, 100):
    start = trajectory[i]
    direction = trajectory[i + 1] - trajectory[i]

    plt.arrow(
        start[0],
        start[1],
        direction[0],
        direction[1],
        head_width=0.08,
        length_includes_head=True
    )



plt.grid()
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.legend()
plt.title("Titulo")
plt.show()


# python3 -m experiments.exp_11_go_to_goal_P_controller