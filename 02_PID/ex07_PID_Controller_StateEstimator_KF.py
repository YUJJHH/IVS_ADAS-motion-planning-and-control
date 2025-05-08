from vehicle_model import VehicleModel
import numpy as np
import matplotlib.pyplot as plt

class PID_Controller(object):
    def __init__(self, reference, measure, step_time, P_Gain=0.6, D_Gain=1.6, I_Gain=0.07):
        self.Kp = P_Gain
        self.Kd = D_Gain
        self.Ki = I_Gain
        self.step_time = step_time
        self.prev_error = reference - measure
        self.integral = 0.0
        self.u = 0.0

    def ControllerInput(self, reference, measure):
        error = reference - measure
        d_error = (error - self.prev_error) / self.step_time
        self.integral += error * self.step_time
        self.u = self.Kp * error + self.Kd * d_error + self.Ki * self.integral
        self.prev_error = error

        
class KalmanFilter:
    def __init__(self, init_val):
        self.x_estimate = np.array([[init_val], [0.0]])  # [position; velocity]
        self.P = np.eye(2)  # 초기 오차 공분산
        self.Q = np.eye(2) * 0.01  # 프로세스 노이즈
        self.R = 0.25  # 측정 노이즈

        # 시스템 모델 (vehicle_model 내부 구조와 맞춤)
        self.A = np.array([[1.0, 0.1], [0.0, 1.0]])
        self.B = np.array([[0.0], [0.1]])
        self.C = np.array([[1.0, 0.0]])  # 측정값은 위치 y만 사용

    def estimate(self, y_measured, u):
        # 예측 단계
        x_pred = self.A @ self.x_estimate + self.B * u
        P_pred = self.A @ self.P @ self.A.T + self.Q

        # 칼만 이득 계산
        S = self.C @ P_pred @ self.C.T + self.R
        K = P_pred @ self.C.T @ np.linalg.inv(S)

        # 업데이트
        y_residual = y_measured - (self.C @ x_pred)
        self.x_estimate = x_pred + K @ y_residual
        self.P = (np.eye(2) - K @ self.C) @ P_pred

        
if __name__ == "__main__":
    target_y = 0.0
    measure_y =[]
    estimated_y = []
    time = []
    step_time = 0.1
    simulation_time = 30   
    plant = VehicleModel(step_time, 0.25, 0.99, 0.05)
    estimator = KalmanFilter(plant.y_measure[0][0])
    controller = PID_Controller(target_y, plant.y_measure[0][0], step_time)
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        measure_y.append(plant.y_measure[0][0])
        estimated_y.append(estimator.x_estimate[0][0])
        estimator.estimate(plant.y_measure[0][0],controller.u)
        controller.ControllerInput(target_y, estimator.x_estimate[0][0])
        plant.ControlInput(controller.u)
    
    plt.figure()
    plt.plot([0, time[-1]], [target_y, target_y], 'k-', label="reference")
    plt.plot(time, measure_y,'r:',label = "Vehicle Position(Measure)")
    plt.plot(time, estimated_y,'c-',label = "Vehicle Position(Estimator)")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
