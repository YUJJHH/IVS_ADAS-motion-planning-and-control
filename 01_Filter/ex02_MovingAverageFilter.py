import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ✅ 클래스 정의 (수정 필요했던 부분)
class MovingAverageFilter:
    def __init__(self, y_initial_measure, num_average=5):  # ✅ 필터 크기 외부 입력 가능하도록 수정
        self.num_average = int(num_average)  # 필터 크기 저장
        self.buffer = [y_initial_measure]    # 초기값을 버퍼에 저장
        self.y_estimate = y_initial_measure  # 초기 추정값

    def estimate(self, y_measure):  # ✅ 평균 계산 로직 추가
        self.buffer.append(y_measure)
        if len(self.buffer) > self.num_average:
            self.buffer.pop(0)  # 버퍼가 넘치면 오래된 데이터 제거
        self.y_estimate = sum(self.buffer) / len(self.buffer)  # 평균 계산
        return self.y_estimate


    
if __name__ == "__main__":
    signal = pd.read_csv("01_filter/Data/example_Filter_1.csv")      
    # signal = pd.read_csv("week_01_filter/Data/example_Filter_2.csv")

    # y_estimate = MovingAverageFilter(signal.y_measure[0])
    
    
    # signal = pd.read_csv("01_Filter/Data/example_Filter_2.csv")      # 측정값 CSV 파일 불러오기

    signal["y_estimate"] = 0.0                             ### ★ 추가: 추정값 저장할 열 생성

    y_estimate = MovingAverageFilter(signal.y_measure[0], num_average=5)  ### ★ 수정: 필터 길이 인자로 전달

    
    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i])
        signal.y_estimate[i] = y_estimate.y_estimate

    plt.figure()
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(signal.time, signal.y_estimate,'r-',label = "Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()



