import numpy as np
import matplotlib.pyplot as plt
from src import math_utils as mu

q = np.array([1.0,3.0])
q_start = q.copy()
q_g = np.array([6.0,4.0])
c_i = [[4.0,3.0], [6.0,2.0], [3.0,5.0]]
c_i = np.array(c_i)
r_i = [0.7, 1.0, 0.6]

rho_0 = 0.5
k_att = 2.5
k_rep = 1.0
v = 0.2
dt = 1
steps = 1000
tol = 0.01
epsilon = 0.08

trajectory = []
trajectory.append(q.copy())

def forces(ro, k_att, k_rep, q, q_g, c, r):
    sum_F_rep = np.array([0.0,0.0])
    for i in range(len(c_i)):
        # FUERZA REPULSIVA
        d = mu.euclidean_distance(c[i],q) - r[i]
        d_safe = max(d, epsilon)
        vector_to_obstacle = q - c[i]
        u = mu.normalize_vector(vector_to_obstacle)
        if d_safe <= ro:
            F_rep = k_rep * ((1/d_safe) - (1/ro)) * (1/(d_safe * d_safe)) * u
            sum_F_rep += F_rep
        else:
            F_rep = np.array([0.0,0.0])

    # FUERZA ATRACTIVA
    vector_to_goal = q_g - q
    d_to_goal = mu.euclidean_distance(q, q_g)

    if d_to_goal > epsilon:
        F_att = k_att * mu.normalize_vector(vector_to_goal)
    else:
        F_att = np.array([0.0, 0.0])

    return F_att + sum_F_rep


def potencial_att(k_att, q, q_g):
    d = mu.euclidean_distance(q, q_g)
    return k_att * d

def potential_rep(ro, k_rep, q, c, r):
    U_rep_total = 0.0

    for i in range(len(c)):
        d = mu.euclidean_distance(c[i], q) - r[i]

        if d <= ro:
            d_safe = max(d, epsilon)

            U_rep_i = 0.5 * k_rep * ((1 / d_safe) - (1 / ro))**2

            U_rep_total += U_rep_i

    return U_rep_total


def potential_total(q_point, k_att, k_rep, q_g, c_i, r_i, rho_0):
    # POTENCIAL ATRACTIVO
    U_att = potencial_att(k_att, q_point, q_g)

    # POTENCIAL REPULSIVO TOTAL
    U_rep_total = potential_rep(rho_0, k_rep, q_point, c_i, r_i)

    return U_rep_total + U_att

for i in range(steps):
    F_t = mu.normalize_vector(forces(rho_0, k_att, k_rep, q, q_g, c_i, r_i))
    q = q + v * F_t * dt
    trajectory.append(q.copy())
    d = mu.euclidean_distance(q, q_g)
    if d <= tol:
        break

trajectory = np.array(trajectory)
# -----------------------------
# MAPA DE POTENCIAL
# -----------------------------

x = np.linspace(-1, 8, 150)
y = np.linspace(-1, 7, 150)
"""
x = np.linspace(-50, 80, 150) # demostracion de malla
y = np.linspace(-50, 70, 150)
"""

X, Y = np.meshgrid(x, y)

Z = np.zeros_like(X)

for row in range(X.shape[0]):
    for col in range(X.shape[1]):
        q_point = np.array([X[row, col], Y[row, col]])

        Z[row, col] = potential_total(
            q_point,
            k_att,
            k_rep,
            q_g,
            c_i,
            r_i,
            rho_0
        )


fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.plot_surface(X, Y, Z, cmap="rainbow", alpha = 0.4)
z_offset = 0.03 * (np.max(Z) - np.min(Z)) # subir un poco la linea para que se vea

z_goal = potential_total(q_g, k_att, k_rep, q_g, c_i, r_i, rho_0)
z_goal = np.array(z_goal)

# Trayectoria
trajectory_z = []
for point in trajectory:
    z_value = potential_total( 
        point,
        k_att,
        k_rep,
        q_g,
        c_i,
        r_i,
        rho_0)
    trajectory_z.append(z_value)
trajectory_z = np.array(trajectory_z)



ax.plot(
    trajectory[:, 0],
    trajectory[:, 1],
    trajectory_z + z_offset,
    label="Trajectory",
    linewidth=3,
    color="red"
)


# Posición inicial del robot
z_start = potential_total(q_start, k_att, k_rep, q_g, c_i, r_i, rho_0)
z_start = np.array(z_start)
ax.scatter(
    q_start[0],
    q_start[1],
    z_start + z_offset,
    label="Robot start",
    color = "black",
    s=10
)

# Goal
ax.scatter(
    q_g[0],
    q_g[1],
    z_goal + z_offset,
    label="Goal",
    color="yellow",
    s=10
)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Potential")
ax.set_title("Artificial potential field - 3D surface")
ax.legend()

# -----------------------------
# MAPA 2D / HEATMAP / CONTOUR MAP
# -----------------------------

plt.figure()

# Opcional: recortar valores muy altos para que los obstáculos no dominen toda la escala
Z_plot = np.clip(Z, 0, 100)

# Mapa de colores del potencial
plt.contourf(
    X,
    Y,
    Z_plot,
    levels=50,
    cmap="rainbow"
)

# Barra de colores
plt.colorbar(label="Potential")

# Líneas de contorno
plt.contour(
    X,
    Y,
    Z_plot,
    levels=25,
    colors="black",
    linewidths=0.3
)

# Trayectoria del robot
plt.plot(
    trajectory[:, 0],
    trajectory[:, 1],
    linewidth=3,
    color="red",
    label="Trajectory"
)

# Posición inicial
plt.scatter(
    trajectory[0, 0],
    trajectory[0, 1],
    s=80,
    label="Robot start",
    color="black"
)

# Posición final
plt.scatter(
    trajectory[-1, 0],
    trajectory[-1, 1],
    s=80,
    label="Robot final",
    color="orange"
)

# Goal
plt.scatter(
    q_g[0],
    q_g[1],
    s=100,
    label="Goal",
    color="yellow"
)

# Obstáculos con sus radios
for i in range(len(c_i)):
    center = c_i[i]
    radius = r_i[i]

    plt.scatter(center[0], center[1])

    circle = plt.Circle(
        center,
        radius,
        fill=False,
        linewidth=2
    )

    plt.gca().add_patch(circle)

    plt.text(
        center[0],
        center[1],
        f"O{i + 1}"
    )

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Artificial potential field - contour map")
plt.axis("equal")
plt.grid()
plt.legend()


# -----------------------------
# CAMPO VECTORIAL DE FUERZAS
# -----------------------------

plt.figure()

# Opcional: usamos el mapa de potencial como fondo
Z_plot = np.clip(Z, 0, 100)
"""
plt.contourf(
    X,
    Y,
    Z_plot,
    levels=50,
    cmap="rainbow",
    alpha=0.7
)

plt.colorbar(label="Potential")
"""
# -----------------------------
# MALLA MÁS PEQUEÑA PARA LAS FLECHAS
# -----------------------------

x_vec = np.linspace(-1, 8, 25)
y_vec = np.linspace(-1, 7, 25)

X_vec, Y_vec = np.meshgrid(x_vec, y_vec)

U = np.zeros_like(X_vec)
V = np.zeros_like(Y_vec)

# -----------------------------
# CALCULAR FUERZA EN CADA PUNTO
# -----------------------------

for row in range(X_vec.shape[0]):
    for col in range(X_vec.shape[1]):

        q_point = np.array([
            X_vec[row, col],
            Y_vec[row, col]
        ])

        F = forces(
            rho_0,
            k_att,
            k_rep,
            q_point,
            q_g,
            c_i,
            r_i
        )

        F_dir = mu.normalize_vector(F)

        U[row, col] = F_dir[0]
        V[row, col] = F_dir[1]

# -----------------------------
# DIBUJAR CAMPO VECTORIAL
# -----------------------------

plt.quiver(
    X_vec,
    Y_vec,
    U,
    V,
    angles="xy",
    scale_units="xy",
    scale=8,
    width=0.003,
    label="Force field"
)

# -----------------------------
# DIBUJAR TRAYECTORIA
# -----------------------------

plt.plot(
    trajectory[:, 0],
    trajectory[:, 1],
    linewidth=3,
    label="Trajectory",
    color="red"
)

plt.scatter(
    trajectory[0, 0],
    trajectory[0, 1],
    s=80,
    label="Robot start",
    color="black"
)

plt.scatter(
    trajectory[-1, 0],
    trajectory[-1, 1],
    s=80,
    label="Robot final",
    color="orange"
)

# -----------------------------
# DIBUJAR GOAL
# -----------------------------

plt.scatter(
    q_g[0],
    q_g[1],
    s=100,
    label="Goal",
    color="yellow"
)

# -----------------------------
# DIBUJAR OBSTÁCULOS
# -----------------------------

for i in range(len(c_i)):
    center = c_i[i]
    radius = r_i[i]

    plt.scatter(center[0], center[1])

    circle = plt.Circle(
        center,
        radius,
        fill=False,
        linewidth=2
    )

    plt.gca().add_patch(circle)

    plt.text(
        center[0],
        center[1],
        f"O{i + 1}"
    )

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Artificial potential field - vector field")
plt.axis("equal")
plt.grid()
plt.legend()

plt.show()
# python3 -m experiments.exp_20_potential_fields_linear
