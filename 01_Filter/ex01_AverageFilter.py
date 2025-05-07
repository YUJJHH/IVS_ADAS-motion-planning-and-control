import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os  # ← 이 줄 추가
class AverageFilter:
    def __init__(self, y_initial_measure):
        self.num_average = 5  # 윈도우 크기 (이동 평균 개수)
        self.buffer = [y_initial_measure]  # 초기 버퍼
        self.y_estimate = y_initial_measure  # 첫 번째 추정값

    def estimate(self, y_measure):
        self.buffer.append(y_measure)
        if len(self.buffer) > self.num_average:
            self.buffer.pop(0)  # 가장 오래된 데이터 제거
        self.y_estimate = sum(self.buffer) / len(self.buffer)

    
if __name__ == "__main__":
    # signal = pd.read_csv("01_Filter/Data/example_Filter_1.csv")
    

    signal = pd.read_csv("01_filter/Data/example_Filter_2.csv")

    y_estimate = AverageFilter(signal.y_measure[0])
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


for i, row in signal.iterrows():
   #print(signal.time[i])
   if (i==0):
       signal.y_estimate[i] = signal.y_measure[i]
   else:
       signal.y_estimate[i] = (signal.y_estimate[i-1])*(num_average-1)/num_average + (signal.y_measure[i])/num_average




