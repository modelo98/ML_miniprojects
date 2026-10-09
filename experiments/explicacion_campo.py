import numpy as np
import matplotlib.pyplot as plt


def potential(q, goal, obstacles, k_att=1.0, k_rep=20.0, rho0=1.8):
    """
    Campo potencial total:
    - Atracción hacia el objetivo.
    - Repulsión cerca de obstáculos circulares.
    """
    q = np.array(q, dtype=float)

    # Potencial atractivo
    U_att = 0.5 * k_att * np.linalg.norm(q - goal) ** 2

    # Potencial repulsivo
    U_rep = 0.0

    for center, radius in obstacles:
        distance_to_center = np.linalg.norm(q - center)

        # Distancia al borde del obstáculo
        clearance = distance_to_center - radius

        # Evitar división por cero si estamos demasiado cerca
        clearance = max(clearance, 1e-6)

        if clearance < rho0:
            U_rep += 0.5 * k_rep * (1.0 / clearance - 1.0 / rho0) ** 2

    return U_att + U_rep


def numerical_gradient(q, goal, obstacles, eps=1e-3):
    """
    Calcula el gradiente numéricamente:

        grad(U) = [dU/dx, dU/dy]

    Usamos diferencias finitas.
    """
    q = np.array(q, dtype=float)
    grad = np.zeros(2)

    for i in range(2):
        dq = np.zeros(2)
        dq[i] = eps

        U_plus = potential(q + dq, goal, obstacles)
        U_minus = potential(q - dq, goal, obstacles)

        grad[i] = (U_plus - U_minus) / (2 * eps)

    return grad


def gradient_descent_navigation(
    start,
    goal,
    obstacles,
    alpha=0.08,
    max_step=0.15,
    tolerance=0.15,
    max_iters=500,
):
    """
    Navegación por descenso de gradiente:

        q_new = q - alpha * grad(U)

    El robot se mueve hacia donde el potencial baja más rápido.
    """
    q = np.array(start, dtype=float)
    path = [q.copy()]

    for _ in range(max_iters):
        grad = numerical_gradient(q, goal, obstacles)

        # Dirección de movimiento: negativo del gradiente
        step = -alpha * grad

        # Limitar el paso máximo
        step_norm = np.linalg.norm(step)

        if step_norm > max_step:
            step = step / step_norm * max_step

        q = q + step
        path.append(q.copy())

        distance_to_goal = np.linalg.norm(q - goal)

        if distance_to_goal < tolerance:
            break

    return np.array(path)


# ============================
# Escenario
# ============================

start = np.array([0.5, 0.5])
goal = np.array([7.5, 5.5])

obstacles = [
    (np.array([3.0, 2.5]), 0.7),
    (np.array([4.5, 4.0]), 0.8),
    (np.array([6.0, 3.0]), 0.6),
]

path = gradient_descent_navigation(start, goal, obstacles)


# ============================
# Crear mapa de potencial
# ============================

x = np.linspace(0, 8, 120)
y = np.linspace(0, 6, 100)

X, Y = np.meshgrid(x, y)
U = np.zeros_like(X)

for i in range(Y.shape[0]):
    for j in range(X.shape[1]):
        q = np.array([X[i, j], Y[i, j]])
        U[i, j] = potential(q, goal, obstacles)

# Recortar potencial para que la gráfica no se sature cerca de obstáculos
U_clipped = np.clip(U, 0, 80)


# ============================
# Visualización
# ============================

fig, ax = plt.subplots(figsize=(8, 6))

contour = ax.contourf(X, Y, U_clipped, levels=40)
fig.colorbar(contour, ax=ax, label="Potencial U(q)")

ax.plot(
    path[:, 0],
    path[:, 1],
    marker="o",
    markersize=3,
    label="Trayectoria por gradiente",
)

ax.scatter(start[0], start[1], s=80, label="Inicio")
ax.scatter(goal[0], goal[1], s=120, marker="*", label="Objetivo")

for center, radius in obstacles:
    circle = plt.Circle(center, radius, fill=False, linewidth=2)
    ax.add_patch(circle)
    ax.scatter(center[0], center[1], marker="x", s=70)

ax.set_title("Navegación con campos potenciales y descenso de gradiente")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_xlim(0, 8)
ax.set_ylim(0, 6)
ax.set_aspect("equal", adjustable="box")
ax.grid(True)
ax.legend(loc="upper left")

plt.show()


# ============================
# Información final
# ============================

print("Número de puntos de la trayectoria:", len(path))
print("Posición final:", path[-1])
print("Distancia final al objetivo:", np.linalg.norm(path[-1] - goal))