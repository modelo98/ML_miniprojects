import numpy as np
import matplotlib.pyplot as plt
from src import math_utils as mu

robot_position = np.array([2.0,1.0])
goal_position = np.array([7.0,5.0])

trajectory =np.array([
    [2.0, 1.0],
    [2.8, 1.4],
    [3.5, 2.0],
    [4.2, 2.8],
    [5.0, 3.5],
    [6.0, 4.3],
    [7.0, 5.0]])

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
robot_goal_distance = mu.euclidean_distance(robot_position,goal_position)
total_distance = 0.0

for i in range(len(trajectory)):
    for j in range(len(obstacles)):
        distance_point_obstacle = mu.euclidean_distance(trajectory[i], obstacles[j])
        clearance = distance_point_obstacle - radius[j]
        if clearance <= 0:
            print("Trajectory point {} collision with obstacle {}".format(i + 1, j + 1))

for i in range(len(trajectory) - 1):
    total_distance += mu.euclidean_distance(trajectory[i], trajectory[i + 1])

print("Total distance:", np.round(total_distance,3))
print("Direct distance:", np.round(robot_goal_distance,3))
# PAINT
plt.scatter(robot_position[0], robot_position[1], label = "Robot position")
plt.scatter(goal_position[0], goal_position[1], label = "Goal position")

for i in range(len(obstacles)):
    plt.scatter(obstacles[i][0], obstacles[i][1], label = "Obstacle {}".format(i + 1))

plt.scatter(trajectory[:, 0], trajectory[:, 1], label="Trajectory points")

for i in range(len(trajectory) - 1):
    plt.plot(
        [trajectory[i][0], trajectory[i + 1][0]],
        [trajectory[i][1], trajectory[i + 1][1]])

plt.plot([robot_position[0], goal_position[0]],
         [robot_position[1], goal_position[1]])

for i in range(len(obstacles)):
    t = np.linspace(0, 2 * np.pi, 100)
    x = obstacles[i][0] + radius[i] * np.cos(t)
    y = obstacles[i][1] + radius[i] * np.sin(t)
    plt.plot(x,y)


plt.grid()
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.legend()
plt.title("Titulo")
plt.show()


# python3 -m experiments.exp_08_robot_trajectory.py
