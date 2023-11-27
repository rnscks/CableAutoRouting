import unittest
from collections import namedtuple
from ReplayMemory import DQNReplayMemory

Transition = namedtuple('Transition', ('state', 'action', 'nextState', 'reward'))

class TestDQNReplayMemory(unittest.TestCase):
    def setUp(self):
        self.memory = DQNReplayMemory(capacity=100)
        
    def test_push(self):
        self.memory.Push(state=1, action=2, nextState=3, reward=4)
        self.assertEqual(len(self.memory), 1)
        
    def test_sample(self):
        self.memory.Push(state=1, action=2, nextState=3, reward=4)
        self.memory.Push(state=3, action=4, nextState=5, reward=6)
        self.memory.Push(state=2, action=3, nextState=4, reward=5)
        
        batchSize = 2
        samples = self.memory.Sample(batchSize)
        
        self.assertEqual(len(samples), batchSize)
        
        
        
if __name__ == '__main__':
    unittest.main()