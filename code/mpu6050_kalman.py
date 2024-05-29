# import sys
# sys.path.append("/home/orangepi/.local/lib/python3.10/site-packages")
# from mpu6050 import mpu6050
# import time
# import math

# class LowPassFilter:
#     def __init__(self, alpha):
#         self.alpha = alpha
#         self.filtered_value = 0

#     def update(self, raw_value):
#         self.filtered_value = self.alpha * raw_value + (1 - self.alpha) * self.filtered_value
#         return self.filtered_value

# class GyroKalmanFilter:
#     def __init__(self, address=0x68):
#         self.accelgyro = mpu6050(address)  # 设置MPU6050的I2C地址为0x68
#         self.now = 0
#         self.lastTime = 0
#         self.dt = 0

#         self.axo = 0
#         self.ayo = 0
#         self.azo = 0
#         self.gxo = 0
#         self.gyo = 0
#         self.gzo = 0

#         self.pi = 3.1415926
#         self.AcceRatio = 16384.0
#         self.GyroRatio = 131.0
#         self.n_sample = 8

#         self.aaxs = [0] * 8
#         self.aays = [0] * 8
#         self.aazs = [0] * 8

#         self.a_x = [0] * 10
#         self.a_y = [0] * 10
#         self.a_z = [0] * 10

#         self.Px = 1
#         self.Py = 1
#         self.Pz = 1

#         alpha = 0.2
#         self.lowpass_filter_ax = LowPassFilter(alpha)
#         self.lowpass_filter_ay = LowPassFilter(alpha)
#         self.lowpass_filter_az = LowPassFilter(alpha)

#     def setup(self):
#         times = 200
#         for i in range(times):
#             accel_data = self.accelgyro.get_accel_data()
#             self.axo += accel_data['x']
#             self.ayo += accel_data['y']
#             self.azo += accel_data['z']
            
#             gyro_data = self.accelgyro.get_gyro_data()
#             self.gxo += gyro_data['x']
#             self.gyo += gyro_data['y']
#             self.gzo += gyro_data['z']
            
#         self.axo /= times
#         self.ayo /= times
#         self.azo /= times
#         self.gxo /= times
#         self.gyo /= times
#         self.gzo /= times

#     def loop(self):
#         self.now = time.time() * 1000
#         self.dt = (self.now - self.lastTime) / 1000.0
#         self.lastTime = self.now

#         aaxs_sum = 0
#         aays_sum = 0
#         aazs_sum = 0

#         accel_data = self.accelgyro.get_accel_data()
#         ax = accel_data['x']
#         ay = accel_data['y']
#         az = accel_data['z']

#         ax = self.lowpass_filter_ax.update(ax)
#         ay = self.lowpass_filter_ay.update(ay)
#         az = self.lowpass_filter_az.update(az)

#         accx = ax / self.AcceRatio
#         accy = ay / self.AcceRatio
#         accz = az / self.AcceRatio

#         if accy == 0 or accz == 0: return None
#         aax = math.atan(accy / accz) * (-180) / self.pi
#         aay = math.atan(accx / accz) * 180 / self.pi
#         aaz = math.atan(accz / accy) * 180 / self.pi

#         for i in range(self.n_sample - 1):
#             self.aaxs[i] = self.aaxs[i + 1]
#             aaxs_sum += self.aaxs[i] * (i + 1)
#             self.aays[i] = self.aays[i + 1]
#             aays_sum += self.aays[i] * (i + 1)
#             self.aazs[i] = self.aazs[i + 1]
#             aazs_sum += self.aazs[i] * (i + 1)

#         self.aaxs[self.n_sample - 1] = aax
#         aaxs_sum += aax * self.n_sample
#         aax = (aaxs_sum / (11 * self.n_sample / 2.0)) * 9 / 7.0
#         self.aays[self.n_sample - 1] = aay
#         aays_sum += aay * self.n_sample
#         aay = (aays_sum / (11 * self.n_sample / 2.0)) * 9 / 7.0
#         self.aazs[self.n_sample - 1] = aaz
#         aazs_sum += aaz * self.n_sample
#         aaz = (aazs_sum / (11 * self.n_sample / 2.0)) * 9 / 7.0

#         gyro_data = self.accelgyro.get_gyro_data()
#         gx = gyro_data['x']
#         gy = gyro_data['y']
#         gz = gyro_data['z']

#         gyrox = - (gx - self.gxo) / self.GyroRatio * self.dt
#         gyroy = - (gy - self.gyo) / self.GyroRatio * self.dt
#         gyroz = - (gz - self.gzo) / self.GyroRatio * self.dt

#         Sx = 0
#         Rx = 0
#         Sy = 0
#         Ry = 0
#         Sz = 0
#         Rz = 0

#         for i in range(10 - 1):
#             self.a_x[i] = self.a_x[i + 1]
#             Sx += self.a_x[i]
#             self.a_y[i] = self.a_y[i + 1]
#             Sy += self.a_y[i]
#             self.a_z[i] = self.a_z[i + 1]
#             Sz += self.a_z[i]

#         self.a_x[9] = aax
#         Sx += aax
#         Sx /= 10
#         self.a_y[9] = aay
#         Sy += aay
#         Sy /= 10
#         self.a_z[9] = aaz
#         Sz += aaz
#         Sz /= 10

#         for i in range(10):
#             Rx += (self.a_x[i] - Sx) ** 2
#             Ry += (self.a_y[i] - Sy) ** 2
#             Rz += (self.a_z[i] - Sz) ** 2

#         Rx /= 9
#         Ry /= 9
#         Rz /= 9

#         self.Px += 0.0025
#         Kx = self.Px / (self.Px + Rx)
#         agx = gyrox + Kx * (aax - gyrox)
#         self.Px = (1 - Kx) * self.Px

#         self.Py += 0.0025
#         Ky = self.Py / (self.Py + Ry)
#         agy = gyroy + Ky * (aay - gyroy)
#         self.Py = (1 - Ky) * self.Py

#         self.Pz += 0.0025
#         Kz = self.Pz / (self.Pz + Rz)
#         agz = gyroz + Kz * (aaz - gyroz)
#         self.Pz = (1 - Kz) * self.Pz

#         return agx, agy, agz, aax, aay, aaz

#     def get_accel_data(self):
#         data = self.loop()
#         while( data == None ):
#             data = self.loop()
#         return data[3:6]


#     def get_gyro_data(self):
#         data = self.loop()
#         while( data == None ):
#             data = self.loop()
#         return data[:3]

# import time

# # 记录第一次接收到数据的时间
# start_time = time.time()

# if __name__ == "__main__":
#     filter = GyroKalmanFilter()
#     filter.setup()
#     while True:
#         print(filter.get_accel_data())
#         print(filter.get_gyro_data())
#         end_time = time.time()
#         time_interval = end_time - start_time
#         print("Time interval since last 'Received':", time_interval, "seconds")

#         # 更新起始时间
#         start_time = end_time
#         # time.sleep(0.1)


# import sys
# sys.path.append("/home/orangepi/.local/lib/python3.10/site-packages")
# from mpu6050 import mpu6050
# import time
# import math
# import numpy as np

# class LowPassFilter:
#     def __init__(self, alpha):
#         self.alpha = alpha
#         self.filtered_value = 0

#     def update(self, raw_value):
#         self.filtered_value = self.alpha * raw_value + (1 - self.alpha) * self.filtered_value
#         return self.filtered_value

# class KalmanFilter:
#     def __init__(self, process_variance, measurement_variance, estimated_error):
#         self.process_variance = process_variance
#         self.measurement_variance = measurement_variance
#         self.estimated_error = estimated_error
#         self.posteri_estimate = 0.0
#         self.posteri_error_estimate = 1.0

#     def update(self, measurement):
#         # Prediction update
#         priori_estimate = self.posteri_estimate
#         priori_error_estimate = self.posteri_error_estimate + self.process_variance

#         # Measurement update
#         blending_factor = priori_error_estimate / (priori_error_estimate + self.measurement_variance)
#         self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
#         self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

#         return self.posteri_estimate

# class GyroKalmanFilter:
#     def __init__(self, process_variance = 0.01, measurement_variance = 5, estimated_error = 1, address=0x68):
#         self.accelgyro = mpu6050(address)  # 设置MPU6050的I2C地址为0x68
#         self.now = 0
#         self.lastTime = 0
#         self.dt = 0

#         self.axo = 0
#         self.ayo = 0
#         self.azo = 0
#         self.gxo = 0
#         self.gyo = 0
#         self.gzo = 0

#         self.pi = 3.1415926
#         self.AcceRatio = 16384.0
#         self.GyroRatio = 131.0
#         self.n_sample = 8

#         self.aaxs = [0] * 8
#         self.aays = [0] * 8
#         self.aazs = [0] * 8

#         self.a_x = [0] * 10
#         self.a_y = [0] * 10
#         self.a_z = [0] * 10

#         self.process_variance = process_variance
#         self.measurement_variance = measurement_variance
#         self.estimated_error = estimated_error

#         self.Px = 1
#         self.Py = 1
#         self.Pz = 1

#         alpha = 1.0
#         self.lowpass_filter_ax = LowPassFilter(alpha)
#         self.lowpass_filter_ay = LowPassFilter(alpha)
#         self.lowpass_filter_az = LowPassFilter(alpha)

#         # Create Kalman filter instances
#         self.kalman_filter_ax = KalmanFilter(process_variance, measurement_variance, estimated_error)
#         self.kalman_filter_ay = KalmanFilter(process_variance, measurement_variance, estimated_error)
#         self.kalman_filter_az = KalmanFilter(process_variance, measurement_variance, estimated_error)

#     def setup(self):
#         times = 200
#         for i in range(times):
#             accel_data = self.accelgyro.get_accel_data()
#             self.axo += accel_data['x']
#             self.ayo += accel_data['y']
#             self.azo += accel_data['z']
            
#             gyro_data = self.accelgyro.get_gyro_data()
#             self.gxo += gyro_data['x']
#             self.gyo += gyro_data['y']
#             self.gzo += gyro_data['z']
            
#         self.axo /= times
#         self.ayo /= times
#         self.azo /= times
#         self.gxo /= times
#         self.gyo /= times
#         self.gzo /= times

#     def loop(self):
#         self.now = time.time() * 1000
#         self.dt = (self.now - self.lastTime) / 1000.0
#         self.lastTime = self.now

#         aaxs_sum = 0
#         aays_sum = 0
#         aazs_sum = 0

#         accel_data = self.accelgyro.get_accel_data()
#         ax = accel_data['x']
#         ay = accel_data['y']
#         az = accel_data['z']

#         # ax = self.lowpass_filter_ax.update(ax)
#         # ay = self.lowpass_filter_ay.update(ay)
#         # az = self.lowpass_filter_az.update(az)

#         ax_filtered = self.kalman_filter_ax.update(ax)
#         ay_filtered = self.kalman_filter_ay.update(ay)
#         az_filtered = self.kalman_filter_az.update(az)

#         accx = ax_filtered / self.AcceRatio
#         accy = ay_filtered / self.AcceRatio
#         accz = az_filtered / self.AcceRatio

#         if accy == 0 or accz == 0: return None
#         aax = math.atan(accy / accz) * (-180) / self.pi
#         aay = math.atan(accx / accz) * 180 / self.pi
#         aaz = math.atan(accz / accy) * 180 / self.pi

#         for i in range(self.n_sample - 1):
#             self.aaxs[i] = self.aaxs[i + 1]
#             aaxs_sum += self.aaxs[i] * (i + 1)
#             self.aays[i] = self.aays[i + 1]
#             aays_sum += self.aays[i] * (i + 1)
#             self.aazs[i] = self.aazs[i + 1]
#             aazs_sum += self.aazs[i] * (i + 1)

#         self.aaxs[self.n_sample - 1] = aax
#         aaxs_sum += aax * self.n_sample
#         aax = (aaxs_sum / (11 * self.n_sample / 2.0)) * 9 / 7.0
#         self.aays[self.n_sample - 1] = aay
#         aays_sum += aay * self.n_sample
#         aay = (aays_sum / (11 * self.n_sample / 2.0)) * 9 / 7.0
#         self.aazs[self.n_sample - 1] = aaz
#         aazs_sum += aaz * self.n_sample
#         aaz = (aazs_sum / (11 * self.n_sample / 2.0)) * 9 / 7.0

#         gyro_data = self.accelgyro.get_gyro_data()
#         gx = gyro_data['x']
#         gy = gyro_data['y']
#         gz = gyro_data['z']

#         gyrox = - (gx - self.gxo) / self.GyroRatio * self.dt
#         gyroy = - (gy - self.gyo) / self.GyroRatio * self.dt
#         gyroz = - (gz - self.gzo) / self.GyroRatio * self.dt

#         Sx = 0
#         Rx = 0
#         Sy = 0
#         Ry = 0
#         Sz = 0
#         Rz = 0

#         for i in range(10 - 1):
#             self.a_x[i] = self.a_x[i + 1]
#             Sx += self.a_x[i]
#             self.a_y[i] = self.a_y[i + 1]
#             Sy += self.a_y[i]
#             self.a_z[i] = self.a_z[i + 1]
#             Sz += self.a_z[i]

#         self.a_x[9] = aax
#         Sx += aax
#         Sx /= 10
#         self.a_y[9] = aay
#         Sy += aay
#         Sy /= 10
#         self.a_z[9] = aaz
#         Sz += aaz
#         Sz /= 10

#         for i in range(10):
#             Rx += (self.a_x[i] - Sx) ** 2
#             Ry += (self.a_y[i] - Sy) ** 2
#             Rz += (self.a_z[i] - Sz) ** 2

#         Rx /= 9
#         Ry /= 9
#         Rz /= 9

#         self.Px += 0.0025
#         Kx = self.Px / (self.Px + Rx)
#         agx = gyrox + Kx * (aax - gyrox)
#         self.Px = (1 - Kx) * self.Px

#         self.Py += 0.0025
#         Ky = self.Py / (self.Py + Ry)
#         agy = gyroy + Ky * (aay - gyroy)
#         self.Py = (1 - Ky) * self.Py

#         self.Pz += 0.0025
#         Kz = self.Pz / (self.Pz + Rz)
#         agz = gyroz + Kz * (aaz - gyroz)
#         self.Pz = (1 - Kz) * self.Pz

#         return agx, agy, agz, aax, aay, aaz

#     def get_accel_data(self):
#         data = self.loop()
#         while data is None:
#             data = self.loop()
#         return data[3:6]

#     def get_gyro_data(self):
#         data = self.loop()
#         while data is None:
#             data = self.loop()
#         return data[:3]

# if __name__ == "__main__":
#     process_variance = 0.01  # 设置过程方差
#     measurement_variance = 0.1  # 设置测量方差
#     estimated_error = 1.0  # 设置初始估计误差

#     filter = GyroKalmanFilter(process_variance, measurement_variance, estimated_error)
#     filter.setup()
#     while True:
#         print(filter.get_accel_data())
#         print(filter.get_gyro_data())


import sys
sys.path.append("/home/orangepi/.local/lib/python3.10/site-packages")
from mpu6050 import mpu6050
import time
import math
import numpy as np

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.filtered_value = 0

    def update(self, raw_value):
        self.filtered_value = self.alpha * raw_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value

class AdaptiveKalmanFilter:
    def __init__(self, initial_process_variance, initial_measurement_variance):
        self.process_variance = initial_process_variance
        self.measurement_variance = initial_measurement_variance
        self.kalman_gain = 0
        self.posteri_estimate = 0
        self.posteri_error_estimate = 1

    def update(self, measurement, residual):
        # Prediction update
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        # Measurement update
        blending_factor = priori_error_estimate / (priori_error_estimate + self.measurement_variance)
        self.kalman_gain = blending_factor
        self.posteri_estimate = priori_estimate + self.kalman_gain * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - self.kalman_gain) * priori_error_estimate

        # Adapt process and measurement variances based on residual
        if residual > 0:
            self.process_variance += (residual - self.process_variance) * 0.1
            self.measurement_variance += (residual - self.measurement_variance) * 0.1

        return self.posteri_estimate

class GyroKalmanFilter:
    def __init__(self, process_variance = 100, measurement_variance = 0.02, estimated_error = 1, address=0x68):
        self.accelgyro = mpu6050(address)  # 设置MPU6050的I2C地址为0x68
        self.now = 0
        self.lastTime = 0
        self.dt = 0

        self.axo = 0
        self.ayo = 0
        self.azo = 0
        self.gxo = 0
        self.gyo = 0
        self.gzo = 0

        self.pi = 3.1415926
        self.AcceRatio = 16384.0
        self.GyroRatio = 131.0
        self.n_sample = 8

        self.aaxs = [0] * 8
        self.aays = [0] * 8
        self.aazs = [0] * 8

        self.a_x = [0] * 10
        self.a_y = [0] * 10
        self.a_z = [0] * 10

        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_error = estimated_error

        self.Px = 1
        self.Py = 1
        self.Pz = 1

        alpha = 0.25
        self.lowpass_filter_ax = LowPassFilter(alpha)
        self.lowpass_filter_ay = LowPassFilter(alpha)
        self.lowpass_filter_az = LowPassFilter(alpha)

        # Create adaptive Kalman filter instances
        self.kalman_filter_ax = AdaptiveKalmanFilter(process_variance, measurement_variance)
        self.kalman_filter_ay = AdaptiveKalmanFilter(process_variance, measurement_variance)
        self.kalman_filter_az = AdaptiveKalmanFilter(process_variance, measurement_variance)

    def setup(self):
        times = 200
        for i in range(times):
            accel_data = self.accelgyro.get_accel_data()
            self.axo += accel_data['x']
            self.ayo += accel_data['y']
            self.azo += accel_data['z']
            
            gyro_data = self.accelgyro.get_gyro_data()
            self.gxo += gyro_data['x']
            self.gyo += gyro_data['y']
            self.gzo += gyro_data['z']
            
        self.axo /= times
        self.ayo /= times
        self.azo /= times
        self.gxo /= times
        self.gyo /= times
        self.gzo /= times

    def loop(self):
        self.now = time.time() * 1000
        self.dt = (self.now - self.lastTime) / 1000.0
        self.lastTime = self.now

        aaxs_sum = 0
        aays_sum = 0
        aazs_sum = 0

        accel_data = self.accelgyro.get_accel_data()
        ax = accel_data['x']
        ay = accel_data['y']
        az = accel_data['z']

        # ax = self.lowpass_filter_ax.update(ax)
        # ay = self.lowpass_filter_ay.update(ay)
        # az = self.lowpass_filter_az.update(az)

        ax_filtered = self.kalman_filter_ax.update(ax, abs(ax - accel_data['x']))
        ay_filtered = self.kalman_filter_ay.update(ay, abs(ay - accel_data['y']))
        az_filtered = self.kalman_filter_az.update(az, abs(az - accel_data['z']))

        accx = ax_filtered / self.AcceRatio
        accy = ay_filtered / self.AcceRatio
        accz = az_filtered / self.AcceRatio

        if accy == 0 or accz == 0: return None
        aax = math.atan(accy / accz) * (-180) / self.pi
        aay = math.atan(accx / accz) * 180 / self.pi
        aaz = math.atan(accz / accy) * 180 / self.pi

        for i in range(self.n_sample - 1):
            self.aaxs[i] = self.aaxs[i + 1]
            aaxs_sum += self.aaxs[i] * (i + 1)
            self.aays[i] = self.aays[i + 1]
            aays_sum += self.aays[i] * (i + 1)
            self.aazs[i] = self.aazs[i + 1]
            aazs_sum += self.aazs[i] * (i + 1)

        self.aaxs[self.n_sample - 1] = aax
        aaxs_sum += aax * self.n_sample
        aax = (aaxs_sum / (11 * self.n_sample / 2.0)) * 9 / 7.0
        self.aays[self.n_sample - 1] = aay
        aays_sum += aay * self.n_sample
        aay = (aays_sum / (11 * self.n_sample / 2.0)) * 9 / 7.0
        self.aazs[self.n_sample - 1] = aaz
        aazs_sum += aaz * self.n_sample
        aaz = (aazs_sum / (11 * self.n_sample / 2.0)) * 9 / 7.0

        gyro_data = self.accelgyro.get_gyro_data()
        gx = gyro_data['x']
        gy = gyro_data['y']
        gz = gyro_data['z']

        gyrox = - (gx - self.gxo) / self.GyroRatio * self.dt
        gyroy = - (gy - self.gyo) / self.GyroRatio * self.dt
        gyroz = - (gz - self.gzo) / self.GyroRatio * self.dt

        Sx = 0
        Rx = 0
        Sy = 0
        Ry = 0
        Sz = 0
        Rz = 0

        for i in range(10 - 1):
            self.a_x[i] = self.a_x[i + 1]
            Sx += self.a_x[i]
            self.a_y[i] = self.a_y[i + 1]
            Sy += self.a_y[i]
            self.a_z[i] = self.a_z[i + 1]
            Sz += self.a_z[i]

        self.a_x[9] = aax
        Sx += aax
        Sx /= 10
        self.a_y[9] = aay
        Sy += aay
        Sy /= 10
        self.a_z[9] = aaz
        Sz += aaz
        Sz /= 10

        for i in range(10):
            Rx += (self.a_x[i] - Sx) ** 2
            Ry += (self.a_y[i] - Sy) ** 2
            Rz += (self.a_z[i] - Sz) ** 2

        Rx /= 9
        Ry /= 9
        Rz /= 9

        self.Px += 0.0025
        Kx = self.Px / (self.Px + Rx)
        agx = gyrox + Kx * (aax - gyrox)
        self.Px = (1 - Kx) * self.Px

        self.Py += 0.0025
        Ky = self.Py / (self.Py + Ry)
        agy = gyroy + Ky * (aay - gyroy)
        self.Py = (1 - Ky) * self.Py

        self.Pz += 0.0025
        Kz = self.Pz / (self.Pz + Rz)
        agz = gyroz + Kz * (aaz - gyroz)
        self.Pz = (1 - Kz) * self.Pz

        return agx, agy, agz, aax, aay, aaz

    def get_accel_data(self):
        data = self.loop()
        while data is None:
            data = self.loop()
        return data[3:6]

    def get_gyro_data(self):
        data = self.loop()
        while data is None:
            data = self.loop()
        return data[:3]

import time

# 记录第一次接收到数据的时间
start_time = time.time()

if __name__ == "__main__":
    process_variance = 0.01
    measurement_variance = 0.1
    estimated_error = 1
    
    filter = GyroKalmanFilter(process_variance, measurement_variance, estimated_error)
    filter.setup()
    while True:
        print(filter.get_accel_data())
        print(filter.get_gyro_data())
        end_time = time.time()
        time_interval = end_time - start_time
        print("Time interval since last 'Received':", time_interval, "seconds")

        # 更新起始时间
        start_time = end_time
        # time.sleep(0.1)

