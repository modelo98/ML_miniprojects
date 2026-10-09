import numpy as np
import matplotlib.pyplot as plt
from src import math_utils as mu

robot_position = np.array([2.0,1.0])
robot_theta = 30
goal_position = np.array([6.0,5.0])
heading = mu.heading_from_angle(robot_theta)
vector_to_goal = goal_position - robot_position
direction_to_goal = mu.normalize_vector(vector_to_goal)
point = np.array([3.0,3.0])

plt.scatter(robot_position[0], robot_position[1], label="Robot")
plt.scatter(goal_position[0], goal_position[1], label="Goal")
plt.scatter(point[0], point[1], label="Point")

plt.arrow(
    robot_position[0],
    robot_position[1],
    heading[0],
    heading[1],

    head_width=0.2,
    length_includes_head=True,
    label="Heading Robot"
)

plt.arrow(
    robot_position[0],
    robot_position[1],
    vector_to_goal[0],
    vector_to_goal[1],
    label="Vector to goal",
    color="red",
    head_width=0.2,
    length_includes_head=True

)

plt.arrow(
    robot_position[0],
    robot_position[1],
    direction_to_goal[0],
    direction_to_goal[1],
    label="Direction to goal",
    color="green",
    head_width=0.2,
    length_includes_head=True

)

plt.plot(
    [robot_position[0], point[0]],
    [robot_position[1],point[1]],
    label="Line to a point"
)


plt.grid()
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.legend()
plt.title("Basic 2D robot visualization")
plt.show()


# python3 -m experiments.exp_06_basic_2d_visualization


