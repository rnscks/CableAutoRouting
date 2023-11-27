
import numpy as np
import random

class VoxelEnvironment:
    def __init__(self) -> None:
        self.InitialState = 0
        self.ActionTable = [1, 4, 2, 3, 5]
        return
    
    def Reset(self):
        return torch.tensor([self.InitialState], dtype=torch.float32)
    
    def Step(self, state, action, step):
        return state + 1, np.random.normal(action, 1, 1), step + 1
        
        