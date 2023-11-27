import numpy as np
import torch

# Define the training data
xTrain = np.linspace(0, 2*np.pi, 100)
print("x linspace",xTrain)   
print("min x linspace value : ",0)   
print("max x linspace value : ",2 * np.pi)    
print("x linspace size", xTrain.size)   
yTrain = np.sin(xTrain)
print("y linspace : ",yTrain)
print("y linspace size : ",yTrain.size) 


# Convert the training data to tensors
xTrain = torch.tensor(xTrain, dtype=torch.float32).unsqueeze(1)

print("xTrain : ",xTrain)   
print("xTrain max value : ",xTrain.max())   
print("xTrain min value : ",xTrain.min())   
print("xTrain size : ",xTrain.size())   
yTrain = torch.tensor(yTrain, dtype=torch.float32).unsqueeze(1)

print("yTrain : ",yTrain)   
print("yTrain max value : ",yTrain.max())   
print("yTrain min value : ",yTrain.min())   
print("yTrain size : ",yTrain.size())   
