import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from GridsMap.GridsMap import GridsMap

# Define the Deep Q-Network (DQN) class
class DQN(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(input_dim, 64)
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, output_dim)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x

    
# Define the GridWorld environment
class DQNEnvirnomentforVoxelWorld:
    def __init__(self, GridsMap: GridsMap):
        self.size = GridsMap.MapSize
        self.board = np.zeros((GridsMap.MapSize, GridsMap.MapSize, GridsMap.MapSize))
        self.board[GridsMap.MapSize-1, GridsMap.MapSize-1, GridsMap.MapSize] = 1
        self.steps = []
        
    def initStep(self):
        for i in range(-1, 2):
            for j in range(-1, 2):
                for k in range(-1, 2):
                    if (i == j == k == 0):
                        continue
                    self.steps.append((i, j, k))
                    
        return

    def reset(self):
        self.board = np.zeros((self.size, self.size))
        self.board[self.size-1, self.size-1, self.size - 1] = 1
        
    def getAction(self, x, y, z, action):
        next_state = (x + self.steps[action][0], y + self.steps[action][1], z + self.steps[action][2])
        return next_state


    def step(self, action):
        x, y, z = np.where(self.board == 1)
        current_state = (x[0], y[0], z[0])

        if self.board[next_state] == 1:  # Reached the goal state
            reward = 100
            done = True
        else:
            reward = -1
            done = False

        self.board[current_state] = 0
        self.board[next_state] = 1

        return next_state, reward, done

# Hyperparameters
gamma = 0.9  # Discount factor
epsilon = 1.0  # Exploration rate
epsilon_min = 0.01  # Minimum exploration rate
epsilon_decay = 0.99  # Decay rate for exploration rate
learning_rate = 0.001  # Learning rate
memory_size = 10000  # Replay buffer size
batch_size = 32  # Batch size

# Initialize the environment and DQN model
env = GridWorld(size=4)
input_dim = env.size * env.size
output_dim = 26  # Number of actions (up, down, left, right)
model = DQN(input_dim, output_dim)
target_model = DQN(input_dim, output_dim)
target_model.load_state_dict(model.state_dict())  # Initialize the target model with the same weights as the main model
optimizer = optim.Adam(model.parameters(), lr=learning_rate)
replay_buffer = []

# Function for selecting an action based on epsilon-greedy policy
def select_action(state, epsilon):
    if np.random.uniform() < epsilon:
        return np.random.randint(output_dim)
    else:
        with torch.no_grad():
            q_values = model(torch.tensor(state).float())
        return torch.argmax(q_values).item()

# Function for updating the DQN model
def update_model():
    if len(replay_buffer) < batch_size:
        return

    samples = np.random.choice(len(replay_buffer), batch_size, replace=False)
    states, actions, rewards, next_states, dones = zip(*[replay_buffer[idx] for idx in samples])

    states = torch.tensor(states).float()
    actions = torch.tensor(actions).long()
    rewards = torch.tensor(rewards).float()
    next_states = torch.tensor(next_states).float()
    dones = torch.tensor(dones).bool()

    q_values = model(states)
    next_q_values = target_model(next_states)
    target_q_values = rewards + gamma * torch.max(next_q_values, dim=1)[0] * ~dones

    q_values = q_values.gather(1, actions.unsqueeze(1)).squeeze()

    loss = nn.MSELoss()(q_values, target_q_values)

    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

# Training loop
num_episodes = 1000
max_steps = 100
episode_rewards = []

for episode in range(num_episodes):
    env.reset()
    state = np.where(env.board == 1)
    state = (state[0][0], state[1][0])
    episode_reward = 0

    for step in range(max_steps):
        action = select_action(state, epsilon)
        next_state, reward, done = env.step(action)
        replay_buffer.append((state, action, reward, next_state, done))
        episode_reward += reward
        state = next_state

        if done:
            break

        if len(replay_buffer) > memory_size:
            replay_buffer.pop(0)

        update_model()

    target_model.load_state_dict(model.state_dict())  # Update the target model weights

    episode_rewards.append(episode_reward)
    epsilon = max(epsilon_min, epsilon * epsilon_decay)

    print(f"Episode: {episode+1}, Reward: {episode_reward}, Epsilon: {epsilon:.4f}")

print("Training complete.")