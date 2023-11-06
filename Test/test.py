import pandas as pd
import matplotlib.pyplot as plt

# Excel 파일을 데이터프레임으로 읽기
df = pd.read_excel('result.xlsx', engine='openpyxl')  # 'sample.xlsx' 파일을 읽고 데이터프레임으로 변환



# 그래프 그리기
plt.figure(figsize=(12, 8))  # 그래프 크기 설정 (선택 사항)
plt.rcParams['font.size'] = 25 


x = []
for i in range(len(df['0waypnt_x'])):
    x.append(i + 1)
 # 글꼴 크기를 14로 설정

plt.xlabel('x').set_weight('bold')
plt.ylabel('y').set_weight('bold')
plt.title('Movement of Way Point 0', fontname ="serif").set_weight('bold')
plt.plot(x, df['0waypnt_x'], color = "k")  
plt.savefig('0waypnt_x.png')
