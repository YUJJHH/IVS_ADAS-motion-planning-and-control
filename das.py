import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os  # ✅ 추가

class AverageFilter:
    def __init__(self, y_initial_measure):
        self.num_average = 5
        self.buffer = [y_initial_measure]
        self.y_estimate = y_initial_measure

    def estimate(self, y_measure):
        self.buffer.append(y_measure)
        if len(self.buffer) > self.num_average:
            self.buffer.pop(0)
        self.y_estimate = sum(self.buffer) / len(self.buffer)

if __name__ == "__main__":
    # ✅ 상대경로 자동 계산
    base_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(base_dir, "Data", "example_Filter_1.csv")
    signal = pd.read_csv(file_path)

    signal["y_estimate"] = 0.0
    y_estimate = AverageFilter(signal.y_measure[0])
    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i])
        signal.at[i, "y_estimate"] = y_estimate.y_estimate

    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measure")
    plt.plot(signal.time, signal.y_estimate, 'r-', label="Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
