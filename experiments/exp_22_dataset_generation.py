import numpy as np
import pandas as pd
from src import math_utils as mu



def random_position():
    x_random = np.random.uniform(-1, 8)
    y_random = np.random.uniform(-1, 7)
    q = np.array([x_random, y_random])
    return q.copy()

# def random_goal():
#     x_random = np.random.uniform(-1, 8)
#     y_random = np.random.uniform(-1, 7)
#     q = np.array([x_random, y_random])
#     return q.copy()

def is_inside_obstacle(position, obstacles_centers, obstacles_radii):
    for i in range(len(obstacles_centers)):
        d = mu.euclidean_distance(position, obstacles_centers[i])
        if d <= obstacles_radii[i]:
            return True
    return False



max_range = 3.0
sensor_names = ["left", "right", "front"]
sensor_colors = ["blue", "green", "red"]
sensor_relative_angles = [45, -45, 0]
obstacles_centers = np.array([
    [4.0, 3.0],
    [6.0, 2.0],
    [3.0, 5.0],
    [1.0, 3.0],
    [2.5, 4.0],
    [5.0, 5.5],
    [7.0, 4.0],
    [0.5, 5.5],
    [4.5, 1.2],
    [6.5, 6.5],
    [2.0, 6.5],
    [8.0, 2.5]
])
obstacles_radii = [0.7, 0.5, 0.7, 0.7, 0.4, 0.6, 0.5, 0.4, 0.5, 0.7, 0.5, 0.6]


# LISTAS PARA GUARDAR DATOS DEL EXP_22
# robot_position
e_22_robot_x = []
e_22_robot_y = []
# goal_position
e_22_goal_x = []
e_22_goal_y = []
# theta
e_22_robot_theta = []
# distance to goal
e_22_distance_to_goal = []
# angle errors
e_22_angle_error = []
# front sensor
e_22_front_sensor = []
# left sensor
e_22_left_sensor = []
# right sensor
e_22_right_sensor = []

e_22_v = []
e_22_omega = []


v = 0.0
dt = 0.01
num_samples = 2000
k_angular = 0.5
k_linear = 0.5
tolerance = 0.01
max_speed = 1.0
omega_max = 2.0
omega_max_deg = np.rad2deg(omega_max)

def caculate_angle_error(robot_position, goal_position, robot_theta):
    vector_to_goal = goal_position - robot_position
    target_angle = np.rad2deg(mu.angle_of_vector(vector_to_goal))
    angle_error = target_angle - robot_theta
    angle_error = mu.normalize_angle_deg(angle_error)
    return angle_error

def calculate_omega(angle_error):
    omega = k_angular * angle_error
    return omega

goal_position = np.array([8.0, 6.0])

while len(e_22_robot_x) < num_samples:

    robot_position = random_position()
    if is_inside_obstacle(robot_position, obstacles_centers, obstacles_radii):
        continue
    robot_theta = np.random.uniform(-180, 180)

    distance_to_goal = mu.euclidean_distance(robot_position,goal_position)
    # Guardar distance to goal, el primero
    # e_22_distance_to_goal.append(distance_to_goal)
    # PEGADO AL OBJETIVO
    if distance_to_goal < tolerance: continue
    # CERCA DEL OBJETIVO
    # if distance_to_goal < 1.0:
    #     max_range = 0.3
    #     k_linear = 0.1

    # CÁLCULO DE SENSORES
    current_detections = []
    sensor_readings = []
    for sensor_idx in range(len(sensor_relative_angles)):

        best_detection = None
        sensor_distance = max_range
        sensor_global_angle = robot_theta + sensor_relative_angles[sensor_idx]
        sensor_direction = mu.heading_from_angle(sensor_global_angle)
        # sensor_end_position = robot_position + sensor_direction * max_range
        # three_sensors_end_position.append(sensor_end_position.copy())

        # Comprobar si el rayo del sensor toca algún obstáculo

        for obstacle_idx in range(len(obstacles_centers)):
            vector_to_obstacle = obstacles_centers[obstacle_idx] - robot_position
            projection = np.dot(vector_to_obstacle,sensor_direction)

            if projection + obstacles_radii[obstacle_idx] < 0 or projection - obstacles_radii[obstacle_idx] > max_range:
                continue

            closest_point = robot_position + projection * sensor_direction
            distance_to_ray = mu.euclidean_distance(closest_point, obstacles_centers[obstacle_idx])

            # El rayo no toca el circulo
            if distance_to_ray > obstacles_radii[obstacle_idx]:
                continue
            
            offset = np.sqrt(obstacles_radii[obstacle_idx]**2 - distance_to_ray**2)
            hit_distance = projection - offset
            hit_point = robot_position + hit_distance * sensor_direction
            if 0 <= hit_distance <= max_range:
                if hit_distance < sensor_distance:
                    sensor_distance = hit_distance
                detection = {
                        "step": len(e_22_robot_x),
                        "sensor": sensor_names[sensor_idx],
                        "obstacle": obstacle_idx + 1,
                        "distance": hit_distance,
                        "hit_point": hit_point.copy(),
                        "closest_point": closest_point.copy()}
                # Guardar hit
                
                

                if best_detection is None or detection["distance"] < best_detection["distance"]:
                    best_detection = detection

        current_detections.append(best_detection)
        sensor_readings.append(sensor_distance)

    left_sensor = sensor_readings[0]
    right_sensor = sensor_readings[1]
    front_sensor = sensor_readings[2]




    left_detected = current_detections[0] is not None
    right_detected = current_detections[1] is not None
    front_detected = current_detections[2] is not None

    angle_error = caculate_angle_error(robot_position, goal_position, robot_theta)
    if left_detected and front_detected:
        v = 0.25
        omega = -90.0
        # robot_theta += omega * dt
        # heading = mu.heading_from_angle(robot_theta)
        # num_left += 1
        # num_front += 1


    elif right_detected and front_detected:
        v = 0.25
        omega = 90.0
        # robot_theta += omega * dt
        # heading = mu.heading_from_angle(robot_theta)
        # num_right += 1
        # num_front += 1
        
    elif right_detected and left_detected:
        v = 0.1
        omega = 0
        # robot_theta += omega * dt
        # heading = mu.heading_from_angle(robot_theta)
        # num_right += 1
        # num_left += 1
       

    elif front_detected:
        v = 0.1
        omega = 50.0
        # robot_theta += omega * dt
        # heading = mu.heading_from_angle(robot_theta)
        # num_front += 1
        

    elif left_detected:
        v = 0.5
        omega = -70.0
        # robot_theta += omega * dt
        # heading = mu.heading_from_angle(robot_theta)
        # num_left += 1
        
    
    elif right_detected:
        v = 0.5
        omega = 70.0
        # robot_theta += omega * dt
        # heading = mu.heading_from_angle(robot_theta)
        # num_right += 1
        
    else:
        v = min(k_linear * distance_to_goal, max_speed) # importante limitar la velocidad al maximo de los motores
        angle_error = caculate_angle_error(robot_position, goal_position, robot_theta)
        # e_22_angle_error.append(np.deg2rad(angle_error))# Guardar angle error
        omega = np.clip(calculate_omega(angle_error), -omega_max_deg, omega_max_deg)
        # robot_theta += omega * dt
        # heading = mu.heading_from_angle(robot_theta)
        
    e_22_robot_x.append(robot_position[0])
    e_22_robot_y.append(robot_position[1])
    e_22_robot_theta.append(np.deg2rad(robot_theta))

    e_22_goal_x.append(goal_position[0])
    e_22_goal_y.append(goal_position[1])

    e_22_distance_to_goal.append(distance_to_goal)
    e_22_angle_error.append(np.deg2rad(angle_error))

    e_22_front_sensor.append(front_sensor)
    e_22_left_sensor.append(left_sensor)
    e_22_right_sensor.append(right_sensor)

    e_22_v.append(v)
    e_22_omega.append(np.deg2rad(omega))

print("Samples:", len(e_22_robot_x))
print("robot_x:", len(e_22_robot_x))
print("robot_y:", len(e_22_robot_y))
print("theta:", len(e_22_robot_theta))
print("goal_x:", len(e_22_goal_x))
print("goal_y:", len(e_22_goal_y))
print("distance:", len(e_22_distance_to_goal))
print("angle_error:", len(e_22_angle_error))
print("front:", len(e_22_front_sensor))
print("left:", len(e_22_left_sensor))
print("right:", len(e_22_right_sensor))
print("v:", len(e_22_v))
print("omega:", len(e_22_omega))


data = {
    "robot_x": e_22_robot_x,
    "robot_y": e_22_robot_y,
    "robot_theta": e_22_robot_theta,
    "goal_x": e_22_goal_x,
    "goal_y": e_22_goal_y,
    "distance_to_goal": e_22_distance_to_goal,
    "angle_error": e_22_angle_error,
    "front_sensor": e_22_front_sensor,
    "left_sensor": e_22_left_sensor,
    "right_sensor": e_22_right_sensor,
    "v": e_22_v,
    "omega": e_22_omega
}

df = pd.DataFrame(data)

print(df.head())
print(df.describe())

df.to_csv("data/navigation_dataset_30.csv", index=False)

# source .venv/bin/activate
# python3 -m experiments.exp_22_dataset_generation