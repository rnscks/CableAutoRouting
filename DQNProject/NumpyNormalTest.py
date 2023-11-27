import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import numpy as np

# 임의의 샘플 데이터 생성 (0부터 3까지의 정수)
data = np.random.randint(0, 4, size=1000)

# 히스토그램 그리기
plt.hist(data, bins=range(5), edgecolor='black', alpha=0.7)

# 축 레이블 추가
plt.xlabel('Number')
plt.ylabel('Frequency')

# 제목 추가
plt.title('Frequency of Numbers from 0 to 3')

# 그래프 표시
plt.show()

plt.show()