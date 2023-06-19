#!/usr/bin/env python3

import math


hz = 200  # IMU frequency


# imu parameters       The more accurate parameters you provide, the better performance
acc_n: float = 0.2  # accelerometer measurement noise standard deviation.
gyr_n: float = 0.05  # gyroscope measurement noise standard deviation.
acc_w: float = 0.02  # accelerometer bias random work noise standard deviation.
gyr_w: float = 4.0e-5  # gyroscope bias random work noise standard deviation.
g_norm: float = 9.81007  # gravity magnitude


print("acc_n: ", acc_n / math.sqrt(hz))
print("gyr_n: ", gyr_n / math.sqrt(hz))
print("acc_w: ", acc_w * math.sqrt(hz))
print("gyr_w: ", gyr_w * math.sqrt(hz))
