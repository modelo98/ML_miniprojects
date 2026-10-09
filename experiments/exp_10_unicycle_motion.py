import numpy as np
import matplotlib.pyplot as plt
from src import math_utils as mu

robot_position = np.array([2.0,1.0])
robot_theta = 0.0
v = 0.5
dt = 0.1
omega = 30.0 # Probar con 10.0 y con 1.0
num_steps =100
heading = mu.heading_from_angle(robot_theta)
trajectory = []
thetas = []
trajectory.append(robot_position.copy())
thetas.append(robot_theta)

for i in range(num_steps):
    heading = mu.heading_from_angle(robot_theta)
    print("Current theta:", robot_theta)

    robot_position[0] += v * heading[0] * dt
    robot_position[1] += v * heading[1] * dt
    robot_theta = robot_theta + (omega * dt)
    trajectory.append(robot_position.copy())
    thetas.append(robot_theta)

    print("-Step", i + 1)
    print("Robot position: {} {}".format(np.round(robot_position[0],5),np.round(robot_position[1],5)))
    print("New theta:", robot_theta)
    print("\n")


trajectory = np.array(trajectory)
print(len(trajectory))
for i in trajectory:
    print(i)



plt.scatter(trajectory[:, 0], trajectory[:, 1], label="Trajectory points")
plt.scatter(trajectory[0][0], trajectory[0][1], label="Initial position")
plt.scatter(trajectory[-1][0], trajectory[-1][1], label="Final position")
plt.plot(
    trajectory[:, 0],
    trajectory[:, 1],
    label="Trajectory line"
)

for i in range(0, len(trajectory) - 1, 5):
    start = trajectory[i]
    direction = trajectory[i + 1] - trajectory[i]

    plt.arrow(
        start[0],
        start[1],
        direction[0],
        direction[1],
        head_width=0.02,
        length_includes_head=True
    )

plt.grid()
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.legend()
plt.title("Titulo")
plt.show()



# python3 -m experiments.exp_10_unicycle_motion