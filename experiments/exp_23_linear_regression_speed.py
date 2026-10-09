import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from src import math_utils as mu

df = pd.read_csv("data/navigation_dataset_30.csv")

dtg = df["distance_to_goal"].to_numpy() # dtg es distance to goal
v = df["v"].to_numpy() # velocidad
print(dtg[:10])
sum_dtg = 0
sum_v = 0
for i in range(len(dtg)):
    sum_dtg += dtg[i]
    sum_v += v[i]
av_dtg = sum_dtg / len(dtg)
av_v = sum_v / len(v)

sum_nom = 0
for i in range(len(dtg)):
    sum_nom += (dtg[i] - av_dtg) * (v[i] - av_v)
sum_den = 0
for i in range(len(dtg)):
    sum_den += ((dtg[i] - av_dtg) * (dtg[i] - av_dtg))

w = sum_nom / sum_den

b = av_v - w * av_dtg

v_pred = w * dtg + b
mse = np.mean((v - v_pred) ** 2)
mae = np.mean(np.abs(v - v_pred))

print(f"W is {w}")                          # corregir la linea de regresion
print(f"b is {b}")
print(f"MSE is {mse}")
print(f"MAE is {mae}")
x = np.linspace(dtg.min(), dtg.max(), 2000) # esta linea deberia ser linea normal, pero no lo es
y = w * x + b
plt.scatter(dtg, v, label="Point", color="red", lw=0.1)
plt.plot(x, y, label="Regression line", lw=4)
plt.xlabel("Distance to goal")
plt.ylabel("Velocity")
plt.title("Correlation")
plt.grid()
plt.show()
# source .venv/bin/activate
# python3 -m experiments.exp_23_linear_regression_speed     