from ex01_AverageFilter import AverageFilter
from ex02_MovingAverageFilter import MovingAverageFilter
from ex03_LowPassFilter import LowPassFilter
from ex04_KalmanFilter import KalmanFilter
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


if __name__ == "__main__":
    t = []
    y_AF = []
    y_MAF = []
    y_LPF = []
    y_KF = []
    
    signal = pd.read_csv("01_Filter/Data/example_KalmanFilter_1.csv")

    #Code
    signal["y_estimate"] = 0.0
    # ✅ 필터 객체 초기화 --------------------------
    y_estimate_AF = AverageFilter(signal.y_measure[0])
    y_estimate_MAF = MovingAverageFilter(signal.y_measure[0], num_average=5)
    y_estimate_LPF = LowPassFilter(signal.y_measure[0], alpha=0.2)
    y_estimate_KF = KalmanFilter(signal.y_measure[0])
    # 
    
    #y_estimate = AverageFilter(signal.y_measure[0])
    
    
    for i, row in signal.iterrows():
        t.append(signal.time[i])
        # Averageg filter
        y_estimate_AF.estimate(signal.y_measure[i])
        y_AF.append(y_estimate_AF.y_estimate)
        # Moving Average filter
        y_estimate_MAF.estimate(signal.y_measure[i])
        y_MAF.append(y_estimate_MAF.y_estimate)
        # Low pass filter
        y_estimate_LPF.estimate(signal.y_measure[i])
        y_LPF.append(y_estimate_LPF.y_estimate)
        # Kalman filter
        y_estimate_KF.estimate(signal.y_measure[i],signal.u[i])
        y_KF.append(y_estimate_KF.x_estimate)
        

    plt.figure()
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(t, y_AF,'m-',label = "Average Filter")
    plt.plot(t, y_MAF,'b-',label = "Moving Average Filter")
    plt.plot(t, y_LPF,'c-',label = "Low Pass Filter")
    plt.plot(t, y_KF,'r-',label = "Kalman Filter")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()



