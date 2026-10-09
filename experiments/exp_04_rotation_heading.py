import numpy as np
from src import math_utils as mu

PI = np.pi
robot_position = np.array([2.0,1.0])
robot_front = np.array([1.0,0.0])
robot_left = np.array([0.0,1.0])
robot_theta = 0.0

theta_1 = 0.0
theta_2 = 45.0
theta_3 = 90.0
theta_4 = 180.0
theta_5 = -90.0

thetas = [theta_1,theta_2,theta_3,theta_4,theta_5]

for i in range(len(thetas)):
    robot_front_new = None
    robot_left_new = None
    heading = mu.heading_from_angle(thetas[i])
    robot_front_new = mu.rotate_vector(robot_front,thetas[i])
    robot_left_new = mu.rotate_vector(robot_left,thetas[i])
    print("The robot's front {} has rotated to {}".format(
        i + 1,
        np.round(robot_front_new,3)
        ))
    print("The robot's left {} has rotated to {}".format(
        i + 1,
        np.round(robot_left_new,3)
        ))
    
    if np.allclose(robot_front_new, heading):
        print("Correct!")
    else:
        print("Different!")

    print("\n")
    #print("Heading {} is {}".format(thetas[i],np.round(heading,3)))


# python -m experiments.exp_04_rotation_heading.py