from collections import deque   
import random

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


replayMemory = ReplayMemory(100)
  
# 모의 미니 배치 데이터 생성
miniBatch = [(1, 'A', 0.1, 2, False), (2, 'B', 0.2, 3, True), (3, 'C', 0.3, 4, False)]

for batch in miniBatch:
    replayMemory.push(batch)
    
# mini_batch에서 데이터 추출
states, actions, rewards, next_states, dones = zip(*replayMemory.sample(3))

# 결과 출력
print("States:", states)
print("Actions:", actions)
print("Rewards:", rewards)
print("Next States:", next_states)
print("Dones:", dones)
