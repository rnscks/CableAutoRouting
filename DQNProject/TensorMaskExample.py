import torch
import torch.nn as nn
import torch.nn.functional as F


# 임의의 상태와 행동을 나타내는 텐서 생성
states = torch.tensor([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
next_states = torch.tensor([[2.0, 3.0], [4.0, 5.0], [0.0, 0.0]])  # 마지막 상태는 [0, 0]으로 가정
non_final_mask = torch.tensor([True, True, False])  # 마지막 상태는 False

next_state_values = torch.zeros(3)
next_state_values[non_final_mask] = 10

print(next_state_values)