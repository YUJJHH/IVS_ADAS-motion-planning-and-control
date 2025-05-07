import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, y_Measure_init, step_time = 0.1, m = 0.1, modelVariance = 0.01, measureVariance = 1.0, errorVariance_init = 10.0):
        self.A = (1.0)
        self.B = step_time/m
        self.C = 1.0
        self.D = 0.0
        self.Q = modelVariance
        self.R = measureVariance
        self.x_estimate = y_Measure_init
        self.P_estimate = errorVariance_init

    def estimate(self, y_measure, input_u):
        # Prediction

        # Update
           ### ⭐ 예측 단계 (Prediction)
        x_predict = self.A * self.x_estimate + self.B * input_u
        P_predict = self.A * self.P_estimate * self.A + self.Q

        ### ⭐ 갱신 단계 (Update)
        K = P_predict * self.C / (self.C * P_predict * self.C + self.R)  # 칼만 이득
        self.x_estimate = x_predict + K * (y_measure - self.C * x_predict)
        self.P_estimate = (1 - K * self.C) * P_predict

        return self.x_estimate

        

if __name__ == "__main__":
    signal = pd.read_csv("01_Filter/Data/example_KalmanFilter_1.csv")
    signal["y_estimate"] = 0.0 
    y_estimate = KalmanFilter(signal.y_measure[0])
    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i],signal.u[i])
        signal.y_estimate[i] = y_estimate.x_estimate

    plt.figure()
    plt.plot(signal.time, signal.y_measure,'k.',label = "Measure")
    plt.plot(signal.time, signal.y_estimate,'r-',label = "Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()



