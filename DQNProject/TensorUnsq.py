import torch

# 1차원 텐서 생성
x = torch.tensor([1, 2, 3, 4])

# 첫 번째 차원에 차원을 추가하여 2차원 텐서로 변환
x_unsqueezed = x.unsqueeze(0)
print(x.numpy())
print(x.shape)  # 기존 텐서의 형태
print(x_unsqueezed.shape)  # 차원이 추가된 텐서의 형태
print(x_unsqueezed)

import random

# 샘플 리스트
my_list = [1, 2, 3, 4, 5]

# 리스트 섞기
random.shuffle(my_list)

# 결과 출력
print(my_list)
