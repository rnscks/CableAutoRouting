import torch
import numpy as np
import torch.nn as nn   
import torch.nn.functional as F 

# Test Code is in Colab
class DQN(nn.Module):
    def __init__(self, stateSize, actionSize):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(stateSize, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, actionSize)
        return
    
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.fc3(x)  
    
    
if (__name__ == '__main__'):
    pass