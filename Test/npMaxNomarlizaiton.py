import numpy as np

m = np.random.rand(3, 4)
m[0][0] = 1e9
m1 = m[m != 1e9]
print(m1)
print(np.max(m1))
m = m / np.max(m1)
print(m)
