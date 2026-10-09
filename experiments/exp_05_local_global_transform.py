import numpy as np
from src import math_utils as mu


robot_position = np.array([2.0,1.0])
robot_theta = 45 # grados

point_1 = np.array([1.0,0.0])
point_2 = np.array([0.0,1.0])
point_3 = np.array([1.0,1.0])
point_4 = np.array([2.0,0.0])

my_points = [point_1,point_2,point_3,point_4]
R = mu.rotation_matrix_2d(robot_theta)

for i in range(len(my_points)):

    point_global = R @ my_points[i] + robot_position
    print("Point {} in local is {} and in global is {}".format(i+1,my_points[i],np.round(point_global,3)))
    point_local = R.T @ (point_global - robot_position)
    print("Point {} in global is {} and in local is {}".format(i+1,np.round(point_global,3),np.round(point_local,3)))
    print("Recovered correctly:", np.allclose(my_points[i], point_local))
    print("\n")





# python3 -m experiments.exp_05_local_global_transform
