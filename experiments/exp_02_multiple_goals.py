import numpy as np
from src.math_utils import euclidean_distance, normalize_vector, scalar_product

robot_position = np.array([0.0,0.0])
heading = np.array([1.0,0.0])
goal_1 = np.array([5.0,5.0])
goal_2 = np.array([-3.0,4.0])
goal_3 = np.array([0.0,0.0])
my_goals = [goal_1,goal_2,goal_3]


closest_goal = None
closest_distance = np.inf
closest_index = None

# python -m experiments.exp_02_multiple_goals
distances = []
for i in range(len(my_goals)):
    
    vector_goal = my_goals[i] - robot_position

    distance = euclidean_distance(robot_position,my_goals[i])
    distances.append(distance)

    normalize_v = normalize_vector(vector_goal)

    goal_position = scalar_product(heading,normalize_v)

    if goal_position >= 0 and distance > 0:
        if distance < closest_distance:
            closest_distance = distance
            closest_goal = my_goals[i]
            closest_index = i + 1

    print("Robot position:", robot_position)
    print("Goal position {}: {}".format(i + 1, my_goals[i]))
    print("Vector to goal {}: {}".format(i + 1, vector_goal))
    print("Distance to goal {}: {}".format(i + 1, distance))
    print("Direction to goal {}: {}".format(i + 1, normalize_v))
    print("Cosine between robot heading and goal direction:", goal_position)

    if 0 <= goal_position <= 0.7:
        print("Goal position is beside")
    elif goal_position > 0.7:
        print("Goal position is in front")
    else:
        print("Goal position is behind")
    
    print("\n")

if closest_goal is not None:
    print("The closest visible goal is Goal {}: {}".format(closest_index, closest_goal))
    print("Distance:", closest_distance)
else:
    print("No visible goal found")