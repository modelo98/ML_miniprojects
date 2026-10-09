import numpy as np
import matplotlib.pyplot as plt
from src import math_utils as mu

# exp_16_reactive_obstacle_avoidance
# exp_17_goal_seeking_with_obstacles

robot_position = np.array([2.0,1.0])
goal_position = np.array([4.0,5.5])
robot_theta = -30.0
max_range = 0.5
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
sensor_end_positions_history = []
sensor_start_positions_history = []
detections_history = []


v = 0.0
dt = 0.01
num_steps = 10000
k_angular = 0.5
k_linear = 0.5
tolerance = 0.01
max_speed = 1.0
# HISTORIAL DE SIMULACIÓN
trajectory = []
thetas = []
speeds = []
angle_errors = []
distances_to_goal = []
steps = []
num_left = 0
num_right = 0
num_front = 0
trajectory.append(robot_position.copy())
thetas.append(robot_theta)

# LOOP / ESCAPE MODE
escape_mode = False
escape_steps = 0
escape_turn_direction = 1

loop_window = 200
min_improvement = 0.05
escape_duration = 120

# FIRST HEADING 
heading = mu.heading_from_angle(robot_theta)



def caculate_angle_error(robot_position, goal_position, robot_theta):
    vector_to_goal = goal_position - robot_position
    target_angle = np.rad2deg(mu.angle_of_vector(vector_to_goal))
    angle_error = target_angle - robot_theta
    angle_error = mu.normalize_angle_deg(angle_error)
    return angle_error

def calculate_omega(angle_error):
    omega = k_angular * angle_error
    return omega

def motion(robot_position, heading, v, dt):
    robot_position[0] = robot_position[0] + v * heading[0] * dt
    robot_position[1] = robot_position[1] + v * heading[1] * dt


for i in range(num_steps):

    distance_to_goal = mu.euclidean_distance(robot_position,goal_position)
    # PEGADO AL OBJETIVO
    if distance_to_goal < tolerance: break
    # CERCA DEL OBJETIVO
    if distance_to_goal < 1.0:
        max_range = 0.3
        k_linear = 0.1

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
                detection = {
                        "step": i,
                        "sensor": sensor_names[sensor_idx],
                        "obstacle": obstacle_idx + 1,
                        "distance": hit_distance,
                        "hit_point": hit_point.copy(),
                        "closest_point": closest_point.copy()}

                if best_detection is None or detection["distance"] < best_detection["distance"]:
                    best_detection = detection

                print("-DETECTION-")
                print("Step:", best_detection["step"])
                print("Sensor:", best_detection["sensor"])
                print("Obstacle:", best_detection["obstacle"])
                print("Distance:", best_detection["distance"])
                print("Hit Point:", best_detection["hit_point"])
                print("Closest point:", best_detection["closest_point"])
                print("\n")

        current_detections.append(best_detection)




    left_detected = current_detections[0] is not None
    right_detected = current_detections[1] is not None
    front_detected = current_detections[2] is not None

    """
    # ESTO NO BORRAR PERO CAMBIAR DE ESTRATEGIA, ESTA NO ME GUSTA, CHOCA CON LA DESACELERACION DEL 
    # ROBOT CUANDO LOS SENSORES DETECTAN ALGO Y CON LA DESACELERACION DEL ROBOT
    # CUANDO SE ACERCA AL OBJETIVO
    # Detectar posible loop: poca mejora en los últimos loop_window steps
    if len(distances_to_goal) >= loop_window and not escape_mode and not distance_to_goal < 1.0:
        old_distance = distances_to_goal[-loop_window]
        current_distance = distance_to_goal

        improvement = old_distance - current_distance

        if improvement < min_improvement:
            escape_mode = True
            escape_steps = escape_duration

            # Elegir dirección de escape
            if left_detected and not right_detected:
                escape_turn_direction = -1   # girar derecha
            elif right_detected and not left_detected:
                escape_turn_direction = 1    # girar izquierda
            else:
                escape_turn_direction = 1    # por defecto izquierda

            print("LOOP DETECTED -> ESCAPE MODE")

    
    if escape_mode:
        v = 0.15
        omega = escape_turn_direction * 90.0
        robot_theta += omega * dt
        heading = mu.heading_from_angle(robot_theta)
        escape_steps -= 1

        if escape_steps <= 0:
            escape_mode = False
            print("ESCAPE MODE OFF")
    """
    if left_detected and front_detected:
        v = 0.25
        omega = -90.0
        robot_theta += omega * dt
        heading = mu.heading_from_angle(robot_theta)
        num_left += 1
        num_front += 1

    elif right_detected and front_detected:
        v = 0.25
        omega = 90.0
        robot_theta += omega * dt
        heading = mu.heading_from_angle(robot_theta)
        num_right += 1
        num_front += 1

    elif right_detected and left_detected:
        v = 0.1
        omega = 0
        robot_theta += omega * dt
        heading = mu.heading_from_angle(robot_theta)
        num_right += 1
        num_left += 1

    elif front_detected:
        v = 0.1
        omega = 50.0
        robot_theta += omega * dt
        heading = mu.heading_from_angle(robot_theta)
        num_front += 1

    elif left_detected:
        v = 0.5
        omega = -70.0
        robot_theta += omega * dt
        heading = mu.heading_from_angle(robot_theta)
        num_left += 1
    
    elif right_detected:
        v = 0.5
        omega = 70.0
        robot_theta += omega * dt
        heading = mu.heading_from_angle(robot_theta)
        num_right += 1

    else:
        v = min(k_linear * distance_to_goal, max_speed) # importante limitar la velocidad al maximo de los motores
        angle_error = caculate_angle_error(robot_position, goal_position, robot_theta)
        omega = calculate_omega(angle_error)
        robot_theta += omega * dt
        heading = mu.heading_from_angle(robot_theta)

    angle_error = caculate_angle_error(robot_position, goal_position, robot_theta)

    if i == 0: minimum_angle_error = abs(angle_error)
    if abs(angle_error) < minimum_angle_error:
        minimum_angle_error = abs(angle_error)

    # MOVER ROBOT
    motion(robot_position, heading, v, dt)
    
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
print("Final step:", steps[-1])
print("Distance start to goal:", initial_distance)
print("Distance final step to goal", final_distance)
#print("Last angle error:", angle_error)
#print("Minimum angle error:", minimum_angle_error)
print("Left times detected:", num_left)
print("Right times detected:", num_right)
print("Front times detected:", num_front)






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

for step in range(0, len(sensor_end_positions_history), 20):

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
"""
plt.figure()

plt.plot(steps, distances_to_goal, label="Distance to goal")
plt.plot(steps, speeds, label="Speed v")
plt.plot(steps, angle_errors, label="Angle error")

plt.grid()
plt.xlabel("Step")
plt.ylabel("Value")
plt.title("Controller variables over time")
plt.legend()
"""
plt.show()

# python3 -m experiments.exp_16_17