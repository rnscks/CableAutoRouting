from collections import namedtuple, deque
import random

Transition = namedtuple('Transition', ('state', 'action', 'nextState', 'reward'))

class DQNReplayMemory:
    def __init__(self, capacity = 10000):
        self.memory = deque([], maxlen = capacity)  
        return
    
    def Push(self, state, action, nextState, reward):   
        self.memory.append(Transition(state = state, action = action, nextState = nextState, reward = reward))
        return             
    
    def Sample(self, batchSize):
        return random.sample(self.memory, batchSize)    
    
    def __len__(self):
        return len(self.memory)     
    
    


if (__name__ == '__main__'):    
    # Create an instance of DQNReplayMemory
    memory = DQNReplayMemory(capacity=100)

    # Push multiple tuples into the memory
    memory.Push(state=1, action=2, nextState=3, reward=4)
    memory.Push(state=3, action=4, nextState=5, reward=6)
    memory.Push(state=2, action=3, nextState=4, reward=5)

    # Sample from the memory
    batchSize = 2
    samples = memory.Sample(batchSize)

    # Print the sampled transitions
    for transition in samples:
        print(transition)
