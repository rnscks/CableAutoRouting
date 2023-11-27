import torch

# 가상의 Q-값 텐서 생성 (예: 4개의 상태에 대한 3개의 행동 각각의 Q-값)
Q_values = torch.tensor([[1.0, 2.0, 3.0],
                         [1.5, 2.5, 3.5],
                         [2.0, 3.0, 4.0],
                         [2.5, 3.5, 4.5]])

# 각 상태에서 선택된 행동의 인덱스 (예: 0, 2, 1, 0)
actions = torch.tensor([[0], [2], [1], [0]])

# gather를 사용하여 각 상태에서 선택된 행동의 Q-값 추출
selected_Q_values = Q_values.gather(1, actions)

print(selected_Q_values)
