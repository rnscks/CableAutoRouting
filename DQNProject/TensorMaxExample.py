import torch

# 임의의 상태 텐서 생성 (예: 4개의 상태, 각 상태는 2개의 특성을 가짐)
states = torch.tensor([[1.0, 2.0], [2.5, 3.0], [3.0, 1.5], [4.5, 2.0]])



# 각 상태에 대해 최대 Q-값 찾기
max_Q_values = states.max(1)[0]

print("Maximum Q-value for each state:\n", max_Q_values)
