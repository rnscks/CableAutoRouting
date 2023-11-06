from abc import ABC, abstractmethod
from overrides import overrides, EnforceOverrides
import abc

# 전략 인터페이스
class SortStrategy(abc.ABC):
    @abc.abstractmethod
    def sort(self, data):
        pass

# 구체적인 전략 클래스들
class QuickSort(SortStrategy):
    def sort(self, data):
        return sorted(data)  # 퀵 정렬

class BubbleSort(SortStrategy):
    def sort(self, data):
        n = len(data)
        for i in range(n):
            for j in range(0, n-i-1):
                if data[j] > data[j+1]:
                    data[j], data[j+1] = data[j+1], data[j]
        return data  # 버블 정렬

# 컨텍스트 클래스
class Sorter:
    def __init__(self, strategy_class):
        self.strategy_class = strategy_class

    def sort(self, data):
        strategy = self.strategy_class()
        return strategy.sort(data)

# 사용 예제
data = [7, 2, 5, 1, 8, 3]

quick_sorter = Sorter(QuickSort)
bubble_sorter = Sorter(BubbleSort)

sorted_data1 = quick_sorter.sort(data)
sorted_data2 = bubble_sorter.sort(data)

print("Quick Sort Result:", sorted_data1)
print("Bubble Sort Result:", sorted_data2)

class Shape(ABC, EnforceOverrides):
    @abstractmethod
    def draw(self) -> None:
        raise NotImplementedError
    
class Circle(Shape):   
    def __init__(self) -> None:
        super().__init__()
        pass    

    @overrides
    def draw(self):
        return super().draw()


cir = Circle()
    
