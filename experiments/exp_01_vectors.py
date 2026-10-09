import numpy as np
from src.math_utils import euclidean_distance, normalize_vector
import matplotlib.pyplot as plt


robot_position = np.array([0.0,0.0])
goal_position = np.array([5.0,5.0])

vector_to_goal = goal_position - robot_position

distance = euclidean_distance(robot_position,goal_position)

normalize_v = normalize_vector(vector_to_goal)

 #distance = √(5² + 5²) = √50 ≈ 7.07


print("Robot position: ", robot_position)
print("Goal position: ", goal_position)
print("Vector to goal: ", vector_to_goal)
print("Distance to goal: ", distance)
print("Direction to goal: ", normalize_v)

plt.scatter(
    robot_position[0], 
    robot_position[1], 
    label="Robot")

plt.scatter(
    goal_position[0], 
    goal_position[1], 
    label="Goal")


plt.arrow(
    robot_position[0],
    robot_position[1],
    vector_to_goal[0],
    vector_to_goal[1],

    head_width=0.2,
    color="blue",
    length_includes_head=True,
    label="Vector"
)

plt.arrow(
    robot_position[0],
    robot_position[1],
    normalize_v[0],
    normalize_v[1],
    label="Direction",
    color="red",
    head_width=0.2,
    length_includes_head=True

)


plt.grid()
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.legend()
plt.title("Basic 2D robot visualization")
plt.show()



# python3 -m experiments.exp_01_vectors

