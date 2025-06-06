import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Lat import VehicleModel_Lat

class PID_Controller_Kinematic(object):
    def __init__(self, step_time, Y_ref, Y_ego, P_Gain=0.2, D_Gain=0.6, I_Gain=0):
        self.dt = step_time
        self.Kp = P_Gain
        self.Kd = D_Gain
        self.Ki = I_Gain

        self.prev_error = Y_ref - Y_ego
        self.integral = 0.0
        self.u = 0.0  # 조향각 출력 (rad)

    def ControllerInput(self, Y_ref, Y_ego):
        error = Y_ref - Y_ego
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt

        # PID 제어 계산
        self.u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

        # [중요] 조향각 제한
        MAX_DELTA = np.deg2rad(30)
        self.u = max(min(self.u, MAX_DELTA), -MAX_DELTA)


    
if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0
    Y_ref = 4.0
    
    time = []
    X_ego = []
    Y_ego = []
    ego_vehicle = VehicleModel_Lat(step_time, Vx)
    controller = PID_Controller_Kinematic(step_time, Y_ref, ego_vehicle.Y)
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)
        controller.ControllerInput(Y_ref, ego_vehicle.Y)
        ego_vehicle.update(controller.u, Vx)

        
    plt.figure(1)
    plt.plot(X_ego, Y_ego,'b-',label = "Position")
    plt.plot([0, X_ego[-1]], [Y_ref, Y_ref], 'k:',label = "Reference")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
#    plt.axis("best")
    plt.grid(True)    
    plt.show()


