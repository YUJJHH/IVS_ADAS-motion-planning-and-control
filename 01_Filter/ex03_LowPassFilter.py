import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class LowPassFilter:
    def __init__(self, y_initial_measure, alpha=0.9):  # ✅ alpha 추가
        self.alpha = alpha                           # ✅ ★ 추가: alpha 저장
        self.y_estimate = y_initial_measure          # ✅ 초기 추정값 저장

    def estimate(self, y_measure):                   # ✅ ★ 수정: LPF 식 구현
        self.y_estimate = self.alpha * self.y_estimate + (1 - self.alpha) * y_measure
        return self.y_estimate



if __name__ == "__main__":
    #signal = pd.read_csv("01_filter/Data/example_Filter_1.csv")
    #signal = pd.read_csv("01_filter/Data/example_Filter_2.csv")      
    signal = pd.read_csv("01_Filter/Data/example_Filter_3.csv")

    signal["y_estimate"] = 0.0                          # ✅ ★ 추가: 추정값 열 추가

    y_estimate = LowPassFilter(signal.y_measure[0], alpha=0.8)  # ✅ ★ alpha 값 조정 가능

    # y_estimate = LowPassFilter(signal.y_measure[0])
    
    
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



