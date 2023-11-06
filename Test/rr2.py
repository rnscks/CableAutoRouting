import numpy as np
import matplotlib.pyplot as plt


def is_pareto_efficient_simple(costs):
    is_efficient = np.ones(costs.shape[0], dtype=bool)
    for i, c in enumerate(costs):
        if is_efficient[i]:
            # Keep any point with a lower cost
            is_efficient[is_efficient] = np.any(
                costs[is_efficient] < c, axis=1)
            is_efficient[i] = True  # And keep self
    return is_efficient


np.random.seed(3)
n_points = 1000
pnts = []
for _ in range(n_points):
    x = np.random.rand()
    y = np.random.rand()
    pnts.append((x, y))

pnts = np.asarray(pnts)
par = is_pareto_efficient_simple(pnts)

for i in range(len(pnts)):
    x, y = pnts[i][0], pnts[i][1]
    if (par[i]):
        plt.scatter(x, y, color="blue", s=100)
    else:
        plt.scatter(x, y, color="red")
plt.show()
