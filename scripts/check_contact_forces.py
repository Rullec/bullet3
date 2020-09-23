import os
import matplotlib.pyplot as plt
import numpy as np
forces = []
with open("log") as f:
    for line in f.readlines():
        line = line.strip()
        if line.find("[solved] FBF contact force") != -1:
            vec = [float(line.split()[5]),
                   float(line.split()[6]),
                   float(line.split()[7])]
            forces.append(np.linalg.norm(vec))

# plt.hist(forces)
plt.title("contact force norm")
plt.plot(forces, label="contact_forces")
plt.plot([0, len(forces)], [1000, 1000], label = "1000N")
plt.legend()
plt.show()