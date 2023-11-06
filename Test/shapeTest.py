import numpy as np
from geneticalgorithm import geneticalgorithm as ga

def f(X):
    return np.sin(X[0]) + np.sin(X[1]) + np.sin(X[2])
    
    
varbound=np.array([[0,10]]*3)

model=ga(function=f,dimension=3,variable_type='real',variable_boundaries=varbound)

model.run()
