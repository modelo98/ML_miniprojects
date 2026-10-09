import numpy as np
import matplotlib.pyplot as plt
from src import math_utils as mu

robot_position = np.array([2.0,1.0])
robot_theta = 45.0
v = 0.5
dt = 1.0
num_steps =10
trajectory = []
heading = mu.heading_from_angle(robot_theta)
distance = 0.0

trajectory.append(robot_position.copy())

for i in range(num_steps):
    robot_position = robot_position + v * heading * dt
    trajectory.append(robot_position.copy())


for i in range(len(trajectory) - 1):
    distance += mu.euclidean_distance(trajectory[i], trajectory[i + 1])


expected_distance = v * dt * num_steps

print("Total distance:", np.round(distance, 3))
print("Expected distance:", np.round(expected_distance, 3))

if np.isclose(distance, expected_distance):
    print("Correct")
else:
    print("Not correct")
    
trajectory = np.array(trajectory)



plt.scatter(trajectory[:, 0], trajectory[:, 1], label="Trajectory points")
plt.scatter(trajectory[0][0], trajectory[0][1], label="Initial position")
plt.scatter(trajectory[-1][0], trajectory[-1][1], label="Final position")
plt.plot(
    trajectory[:, 0],
    trajectory[:, 1],
    label="Trajectory line"
)
plt.arrow(
    trajectory[0][0],
    trajectory[0][1],
    trajectory[1][0] - trajectory[0][0],
    trajectory[1][1] - trajectory[0][1],
    head_width = 0.1
)
plt.grid()
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.legend()
plt.title("Titulo")
plt.show()

# python3 -m experiments.exp_09_simple_motion.py