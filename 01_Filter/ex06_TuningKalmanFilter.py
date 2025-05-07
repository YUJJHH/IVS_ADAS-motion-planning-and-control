import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, y_Measure_init, step_time=0.1, m=0.1, Q=0.01, R=0.1):
        self.dt = step_time
        self.m = m
        self.A = 1  # 상태 전이 행렬
        self.B = self.dt / self.m  # 입력에 대한 계수
        self.C = 1  # 측정 행렬

        self.Q = Q  # 프로세스 노이즈 공분산
        self.R = R  # 측정 노이즈 공분산

        self.x_estimate = y_Measure_init  # 초기 추정값
        self.P = 1.0  # 초기 공분산

    def estimate(self, y_measure, input_u):
        # 예측 단계
        x_predict = self.A * self.x_estimate + self.B * input_u
        P_predict = self.A * self.P * self.A + self.Q

        # 칼만 이득 계산
        K = P_predict * self.C / (self.C * P_predict * self.C + self.R)

        # 업데이트 단계
        self.x_estimate = x_predict + K * (y_measure - self.C * x_predict)
        self.P = (1 - K * self.C) * P_predict

        

if __name__ == "__main__":
    signal = pd.read_csv("01_Filter/Data/example06.csv")

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



