import numpy as np
import matplotlib.pyplot as plt
from src import math_utils as mu

# WITH P-CONTROLLER

robot_position = np.array([2.0,1.0])
waypoints = np.array([
    [4.0, 2.0],
    [6.0, 5.0],
    [3.0, 7.0],
    [8.0, 8.0]])
way_i = 0
robot_theta = -45.0 
v = 0.0
dt = 0.01
num_steps = 10000
k_angular = 3
k_linear = 2
trajectory = []
thetas = []
speeds = []
angle_errors = []
distances_goal = []
steps = []
trajectory.append(robot_position.copy())
thetas.append(robot_theta)
tolerance = 0.01
max_speed = 1.0
heading = mu.heading_from_angle(robot_theta)

for i in range(num_steps):

    distance_to_goal = mu.euclidean_distance(robot_position,waypoints[way_i])

    if distance_to_goal < tolerance: 
        way_i += 1
        if way_i == len(waypoints):
            break
        continue

    v = min(k_linear * distance_to_goal, max_speed) # importante limitar la velocidad al maximo de los motores
    vector_to_goal = waypoints[way_i] - robot_position
    angle_error = mu.angle_error(vector_to_goal, robot_theta)
    omega = k_angular * angle_error
    robot_position[0] = robot_position[0] + v * heading[0] * dt
    robot_position[1] = robot_position[1] + v * heading[1] * dt
    robot_theta += omega * dt
    heading = mu.heading_from_angle(robot_theta)
    if i == 0: minimum_angle_error = abs(angle_error)
    if abs(angle_error) < minimum_angle_error:
        minimum_angle_error = abs(angle_error)

    steps.append(i)
    speeds.append(v)
    angle_errors.append(angle_error)
    distances_goal.append(distance_to_goal)
    trajectory.append(robot_position.copy())
    thetas.append(robot_theta)

    print("-Step", i)
    print("Robot position: {} {}".format(np.round(robot_position[0],5),np.round(robot_position[1],5)))
    print("New theta:", robot_theta)
    print("Angle error:", angle_error)
    print("Minimum, angle error:", minimum_angle_error)
    print("Current speed:", v)
    print("\n")


trajectory = np.array(trajectory)
initial_distance = mu.euclidean_distance(trajectory[0],waypoints[-1])
final_distance = mu.euclidean_distance(trajectory[-1],waypoints[-1])
print("Distance start to goal:", initial_distance)
print("Distance final step to goal", final_distance)
print("Last angle error:", angle_error)
print("Minimum angle error:", minimum_angle_error)

plt.scatter(trajectory[:, 0], trajectory[:, 1], label="Trajectory points")
plt.scatter(waypoints[:, 0], waypoints[:, 1], label="Way points")
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

plt.figure()

plt.plot(steps, distances_goal, label="Distance to goal")
plt.plot(steps, speeds, label="Speed v")
plt.plot(steps, angle_errors, label="Angle error")

plt.grid()
plt.xlabel("Step")
plt.ylabel("Value")
plt.title("Controller variables over time")
plt.legend()
plt.show()

# python3 -m experiments.exp_13_waypoint_navigation