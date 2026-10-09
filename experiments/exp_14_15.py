import numpy as np
import matplotlib.pyplot as plt
from src import math_utils as mu

# TOGETHER
# exp_14_sensor_rays
# exp_15_obstacle_detection

robot_position = np.array([2.0,1.0])
goal_position = np.array([2.5,4.0])
robot_theta = -30.0
max_range = 3.0
sensor_names = ["left", "right", "front"]
sensor_colors = ["blue", "green", "red"]
sensor_relative_angles = [45, -45, 0]
obstacles_centers = np.array([
    [4.0,3.0],
    [6.0,2.0],
    [3.0,5.0],
    [1.0,3.0]
])
obstacles_radii = [0.2,0.5,0.4,0.3]
sensor_end_positions_history = []
sensor_start_positions_history = []
detections_history = []


v = 0.0
dt = 0.01
num_steps = 5000
k_angular = 0.5
k_linear = 0.5
# HISTORIAL DE SIMULACIÓN
trajectory = []
thetas = []
speeds = []
angle_errors = []
distances_to_goal = []
steps = []
trajectory.append(robot_position.copy())
thetas.append(robot_theta)
tolerance = 0.01
max_speed = 1.0
heading = mu.heading_from_angle(robot_theta)

for i in range(num_steps):

    distance_to_goal = mu.euclidean_distance(robot_position,goal_position)
    if distance_to_goal < tolerance: break
    v = min(k_linear * distance_to_goal, max_speed) # importante limitar la velocidad al maximo de los motores
    vector_to_goal = goal_position - robot_position
    target_angle = np.rad2deg(mu.angle_of_vector(vector_to_goal))
    angle_error = target_angle - robot_theta
    angle_error = mu.normalize_angle_deg(angle_error)
    omega = k_angular * angle_error
    robot_position[0] = robot_position[0] + v * heading[0] * dt
    robot_position[1] = robot_position[1] + v * heading[1] * dt
    robot_theta += omega * dt
    heading = mu.heading_from_angle(robot_theta)
    if i == 0: minimum_angle_error = abs(angle_error)
    if abs(angle_error) < minimum_angle_error:
        minimum_angle_error = abs(angle_error)

    # CÁLCULO DE SENSORES
    three_sensors_end_position = []
    current_detections = []

    for sensor_idx in range(len(sensor_relative_angles)):
        best_detection = None

        sensor_global_angle = robot_theta + sensor_relative_angles[sensor_idx]
        sensor_direction = mu.heading_from_angle(sensor_global_angle)
        sensor_end_position = robot_position + sensor_direction * max_range
        three_sensors_end_position.append(sensor_end_position.copy())

        # Comprobar si el rayo del sensor toca algún obstáculo

        for obstacle_idx in range(len(obstacles_centers)):
            vector_to_obstacle = obstacles_centers[obstacle_idx] - robot_position
            projection = np.dot(vector_to_obstacle,sensor_direction)

            if projection < 0 or projection > max_range:
                continue

            closest_point = robot_position + projection * sensor_direction
            distance_to_ray = mu.euclidean_distance(closest_point, obstacles_centers[obstacle_idx])

            # dentro del area del circulo
            if distance_to_ray <= obstacles_radii[obstacle_idx]:
                detection = {
                    "step": i,
                    "sensor": sensor_names[sensor_idx],
                    "obstacle": obstacle_idx + 1,
                    "distance": projection,
                    "closest_point": closest_point.copy()
                }
                
                if best_detection is None or projection < best_detection["distance"]:
                    best_detection = detection

                print("-DETECTION-")
                print("Step:", best_detection["step"])
                print("Sensor:", best_detection["sensor"])
                print("Obstacle:", best_detection["obstacle"])
                print("Distance:", best_detection["distance"])
                print("Closest point:", best_detection["closest_point"])
                print("\n")

        current_detections.append(best_detection)
    
    # Guardar cosas en historiales
    detections_history.append(current_detections)
    sensor_end_positions_history.append(three_sensors_end_position)
    sensor_start_positions_history.append(robot_position.copy())
    steps.append(i)
    speeds.append(v)
    angle_errors.append(angle_error)
    distances_to_goal.append(distance_to_goal)
    trajectory.append(robot_position.copy())
    thetas.append(robot_theta)

    sensor_print = detections_history[i] 




trajectory = np.array(trajectory)
initial_distance = mu.euclidean_distance(trajectory[0],goal_position)
final_distance = mu.euclidean_distance(trajectory[-1],goal_position)
print("Distance start to goal:", initial_distance)
print("Distance final step to goal", final_distance)
print("Last angle error:", angle_error)
print("Minimum angle error:", minimum_angle_error)
for m in detections_history:
    for n in m:
        print(n)

plt.scatter(trajectory[:, 0], trajectory[:, 1], label="Trajectory points")
plt.scatter(trajectory[0][0], trajectory[0][1], label="Initial position")
plt.scatter(trajectory[-1][0], trajectory[-1][1], label="Final position")
plt.plot(
    trajectory[:, 0],
    trajectory[:, 1],
    label="Trajectory line"
)

for i in range(0, len(trajectory) - 1, 100):
    start = trajectory[i]
    direction = trajectory[i + 1] - trajectory[i]

    plt.arrow(
        start[0],
        start[1],
        direction[0],
        direction[1],
        head_width=0.08,
        length_includes_head=True
    )

# SENSORS   

for step in range(0, len(sensor_end_positions_history), 50):

    sensor_start = sensor_start_positions_history[step]

    for sensor_idx in range(len(sensor_relative_angles)):

        sensor_ON_OFF = detections_history[step][sensor_idx]
        sensor_end = sensor_end_positions_history[step][sensor_idx]
        if sensor_ON_OFF is not None:
            plt.plot(
                [sensor_start[0], sensor_end[0]],
                [sensor_start[1], sensor_end[1]],
                color=sensor_colors[sensor_idx],
                linestyle="-",
                alpha=0.6,
                label=sensor_names[sensor_idx] if step == 0 else None
            )

        else:
            plt.plot(
                [sensor_start[0], sensor_end[0]],
                [sensor_start[1], sensor_end[1]],
                color=sensor_colors[sensor_idx],
                linestyle="--",
                alpha=0.6,
                label=sensor_names[sensor_idx] if step == 0 else None
            )

# Obstáculos con centro y radio
for i in range(len(obstacles_centers)):
    center = obstacles_centers[i]
    radius = obstacles_radii[i]

    # Centro del obstáculo
    plt.scatter(center[0], center[1])

    # Círculo del obstáculo
    circle = plt.Circle(
        center,
        radius,
        fill=False
    )

    plt.gca().add_patch(circle)


    # Texto
    plt.text(
        center[0],
        center[1],
        f"O{i + 1}"
    )


plt.grid()
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.legend()
plt.title("Moving robot with range sensor rays")

plt.figure()

plt.plot(steps, distances_to_goal, label="Distance to goal")
plt.plot(steps, speeds, label="Speed v")
plt.plot(steps, angle_errors, label="Angle error")

plt.grid()
plt.xlabel("Step")
plt.ylabel("Value")
plt.title("Controller variables over time")
plt.legend()
plt.show()

# python3 -m experiments.exp_14_15