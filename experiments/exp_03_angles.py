import numpy as np
from src import math_utils as mu

PI = np.pi
robot_position = np.array([2.0,1.0])
robot_theta = PI / 4.0
goal_1 = np.array([6.0,5.0])
goal_2 = np.array([2.0,6.0])
goal_3 = np.array([-2.0,1.0])
goal_4 = np.array([5.0,-2.0])
goal_5 = np.array([5.0,-2.0])
goal_6 = np.array([2.0,1.0])
my_goals = [goal_1,goal_2,goal_3,goal_4,goal_5,goal_6]

# python -m experiments.exp_03_angles

for i in range(6):

    vector_goal = my_goals[i] - robot_position

    distance = mu.euclidean_distance(robot_position,my_goals[i])

    normalize_v = mu.normalize_vector(vector_goal)

    #goal_position = mu.scalar_product(heading,normalize_v)

    target_angle = mu.angle_of_vector(vector_goal)

    error_angle = mu.angle_between_vectors(target_angle,robot_theta)
    angle_error_normalized = mu.normalize_angle(error_angle)
    print("Goal", i + 1)
    print("Vector to goal:", vector_goal)
    print("Direction to goal(norm):", normalize_v)
    print("Target angle with X-axis(rad):", target_angle)
    print("Target angle with X-axis(º):",np.rad2deg(target_angle))
    print("Error angle:", error_angle)
    print("Error angle:", np.rad2deg(error_angle))
    print("(norm) Angle_error:", angle_error_normalized)
    print("(norm) Angle_error:", np.rad2deg(angle_error_normalized))
    if angle_error_normalized < 0:
        print("Move to the right")
    elif angle_error_normalized == 0:
        print("Go straight")
    else:
        print("Move to the left")

    print("\n")
