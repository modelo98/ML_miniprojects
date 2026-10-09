import random
import numpy as np
import matplotlib.pyplot as plt

alpha = 0.01
generate_list = []
filter_list = []
for step in range(100):
    random_float = random.uniform(-0.5, 0.5)
    generate_list.append(random_float)
    if step == 0:
        filter = alpha * random_float
        previous = filter
        filter_list.append(filter)
        continue
    filter = alpha * random_float + (1 - alpha) * previous
    previous = filter
    print(previous)
    filter_list.append(filter)

plt.figure()
step = range(100)
plt.plot(step, generate_list, label="Data")
plt.plot(step, filter_list, label="Filtered")

plt.grid()
plt.xlabel("Step")
plt.ylabel("Value")
plt.title("Controller variables over time")
plt.legend()
plt.show()