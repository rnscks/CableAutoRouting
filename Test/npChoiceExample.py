import numpy as np
import matplotlib.pyplot as plt

array = np.array([0, 1, 2, 3, 4])
probabilities = np.array([0.1, 0.3, 0.2, 0.2, 0.2])

x = [i for i in range(len(array))]
frequency = [0 for _ in range(1, len(array) + 1)]

for _ in range(1000):
    randomElement = np.random.choice(array, p=probabilities)
    frequency[randomElement] += 1

plt.bar(x, frequency)

plt.show()
