import unittest
from collections import namedtuple  

Transition = namedtuple('Transition', ('state', 'action', 'nextState', 'reward'))


class TestNamedTupleCreation(unittest.TestCase):
    def test_namedtuple_creation(self):
        transition = Transition(state = 1, action = 2, nextState = 3, reward = 4)
        self.assertEqual(transition.state, 1)
        self.assertEqual(transition.action, 2)
        self.assertNotEqual(transition.action, 3)
        
        self.assertEqual(transition.nextState, 3)
        self.assertEqual(transition.reward, 4)   
        print("NamedTuple creation test passed!")
