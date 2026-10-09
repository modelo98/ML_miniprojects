import numpy as np
import matplotlib.pyplot as plt
from src import math_utils as mu

robot_position = np.array([2.0,1.0])
robot_theta = 45
goal_position = np.array([7.0,5.0])
heading = mu.heading_from_angle(robot_theta)

obstacle_1_center = np.array([4.0,3.0])
radius_1 = 0.7
obstacle_2_center = np.array([6.0,2.0])
radius_2 = 1.0
obstacle_3_center = np.array([3.0,5.0])
radius_3 = 0.6
obstacle_4_center = np.array([1.0,3.0])
radius_4 = 0.5

obstacles = [obstacle_1_center,obstacle_2_center,obstacle_3_center,obstacle_4_center]
radius = [radius_1,radius_2,radius_3,radius_4]
my_vectors_obstacles = []

for i in range(len(obstacles)):
    distance_to_center = mu.euclidean_distance(robot_position,obstacles[i])
    clearance = distance_to_center - radius[i]

    if clearance > 2.0:
        print("Obstacle", i + 1)
        print("Distance to center:", np.round(distance_to_center, 3))
        print("Clearance:", np.round(clearance, 3))
        print("\n")
    elif 0.5 < clearance <= 2.0:
        print("Obstacle", i + 1)
        print("Distance to center:", np.round(distance_to_center, 3))
        print("Clearance:", np.round(clearance, 3))
        print("\n")
    elif clearance <= 0.5:
        print("Obstacle", i + 1)
        print("Distance to center:", np.round(distance_to_center, 3))
        print("Clearance:", np.round(clearance, 3))
        print("\n")

    vector_to_goal = obstacles[i] - robot_position
    direction_to_goal = mu.normalize_vector(vector_to_goal)
    vector_to_obstacle = direction_to_goal * clearance
    my_vectors_obstacles.append(vector_to_obstacle)


plt.scatter(robot_position[0], robot_position[1], label="Robot position")
plt.scatter(goal_position[0], goal_position[1], label="Goal position")

for i in range(len(obstacles)):
    plt.scatter(obstacles[i][0], obstacles[i][1], label="Obstacle {}".format(i+1))

    t = np.linspace(0, 2 * np.pi, 100)
    x = obstacles[i][0] + radius[i] * np.cos(t)
    y = obstacles[i][1] + radius[i] * np.sin(t)
    plt.plot(x,y)

    plt.arrow(
        robot_position[0],
        robot_position[1],
        my_vectors_obstacles[i][0],
        my_vectors_obstacles[i][1],
        label="Vector to obstacle {}".format(i+1),
        head_width=0.1,
        length_includes_head=True

    )

plt.arrow(
    robot_position[0],
    robot_position[1],
    heading[0],
    heading[1],
    head_width=0.15,
    length_includes_head=True,
    label="Robot heading"
)

plt.grid()
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.legend()
plt.title("2D robot visualization with circular obstacles")
plt.show()

# python3 -m experiments.exp_07_obstacles_visualization
