from collections import namedtuple  

Transition = namedtuple('Transition', ('state', 'action', 'nextState', 'reward'))

transition = Transition(state = 1, action = 2, nextState = 3, reward = 4)

print(transition.state)
print(transition.action)
print(transition.nextState)
print(transition.reward)