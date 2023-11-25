import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import matplotlib.pyplot as plt
import random
from collections import deque


# 0 ~ 10 state
class Environment: 
    def __init__(self) -> None:
        
        pass
    
    def reset(self):
        return 0, False
    

# Define your DQN class
class DQN(nn.Module):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, 24)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(24, 24)
        self.fc3 = nn.Linear(24, action_size)

    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        return self.fc3(x)

# Define your DQNAgent class
class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.learningRate = 0.001
        self.discountFactor = 0.99
        self.epsilonDecay = 0.999
        self.epsilonMin = 0.01
        self.epsilon = 1.0

        self.memory = ReplayMemory(200)
        self.batchSize = 10
        self.deepNetWorkModel = DQN(state_size, action_size)
        self.targetDeepNetWorkModel = DQN(state_size, action_size)
        self.optimizer = optim.Adam(self.deepNetWorkModel.parameters(), lr=0.001)
        self.lossFunction = nn.MSELoss()

    def GetAction(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        q_values = self.deepNetWorkModel(torch.tensor(state, dtype=torch.float32))
        action = torch.argmax(q_values).item()
        return action

    def AppendSample(self, state, action, reward, next_state, done):
        self.memory.push((state, action, reward, next_state, done))
        
    def trainModel(self):
        if len(self.memory) < self.batchSize:
            return

        mini_batch = self.memory.sample(self.batchSize)
        states, actions, rewards, nextStates, dones = zip(*mini_batch)

        states = torch.tensor(states, dtype=torch.int64)
        actions = torch.tensor(actions, dtype=torch.int64)
        rewards = torch.tensor(rewards, dtype=torch.float32)
        nextStates = torch.tensor(nextStates, dtype=torch.int64)
        dones = torch.tensor(dones, dtype=torch.uint8)

        qValues = self.deepNetWorkModel(states)
        targetQValues = self.targetDeepNetWorkModel(nextStates)

        targetValues = torch.max(targetQValues, dim=1)[0]
        targetValues[dones] = 0

        targetQ = rewards + self.discountFactor * targetValues

        selectedQValues = qValues.gather(1, actions.view(-1, 1)).squeeze()

        loss = self.lossFunction(selectedQValues, targetQ.detach())

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()


class ReplayMemory:
    def __init__(self, capacity) -> None:
        self.memory = deque([], maxlen=capacity) 
        pass    
    
    def push(self, args):
        self.memory.append(args)
        pass    
    
    def sample(self, batchSize):
        return random.sample(self.memory, batchSize)

    def __len__(self) -> int:
        return len(self.memory)
    
    

agent = DQNAgent(1, 4)
env = Environment()
numberofEpisodes = 100

for episode in range(numberofEpisodes):
    state, _ = env.reset()
    for i in range(10):
        action = agent.GetAction(state)
            
agent.trainModel()