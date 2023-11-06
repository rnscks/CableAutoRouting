import pandas as pd
import matplotlib.pyplot as plt


# 그래프 표시 (선택 사항)
plt.show()

# Excel 파일을 데이터프레임으로 읽기
df = pd.read_excel('result.xlsx', engine='openpyxl')  # 'sample.xlsx' 파일을 읽고 데이터프레임으로 변환
# 그래프 그리기
plt.figure(figsize=(8, 6))  # 그래프 크기 설정 (선택 사항)

plt.bar(df['월'], df['판매량'])  # 막대 그래프를 그립니다.
plt.xlabel('월')
plt.ylabel('판매량')
plt.title('월별 판매량 그래프')

# 데이터프레임 확인
print(df)
