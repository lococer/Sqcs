# This code is used to estimate the posture of the robot using the MPU6050 sensor.
# using complementary filter to combine the information from the gyroscope and accelerometer.
# using getdata() function to get the current posture of the robot.

import time
import sys
sys.path.append("/home/orangepi/.local/lib/python3.10/site-packages")
# from mpu6050 import mpu6050
from mpu6050_kalman import GyroKalmanFilter
import math
import threading

class posture:
    class LowPassFilter: #low pass filter
        def __init__(self, alpha, special=False):
            self.alpha = alpha
            self.filtered_value = None
            self.sepcial = special

        def update(self, new_value):
            if self.filtered_value is None:
                self.filtered_value = new_value
            else:
                if self.sepcial:
                    error = new_value - self.filtered_value
                    if abs(error) < 10:
                        self.filtered_value = self.filtered_value * 0.01
                    else:
                        self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value
                else:
                    error = new_value - self.filtered_value
                    if abs(error) < 10:
                        self.filtered_value = self.filtered_value * 0.01
                    else:
                        self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value
            return self.filtered_value
        
    def __init__(self, dt):
        # self.sensor = mpu6050(0x68)
        self.sensor = GyroKalmanFilter(0x68)
        self.dt = dt  # 采样时间间隔（秒）
        self.alpha = 0.98  # 陀螺仪权重，加速度计权重为 1 - alpha
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        lowpassalpha = 0.3  # 低通滤波器参数
        self.gyro_x_filter = self.LowPassFilter(lowpassalpha, True)
        self.gyro_y_filter = self.LowPassFilter(lowpassalpha)
        self.gyro_z_filter = self.LowPassFilter(lowpassalpha)

        self.accel_x_filter = self.LowPassFilter(lowpassalpha)
        self.accel_y_filter = self.LowPassFilter(lowpassalpha)
        self.accel_z_filter = self.LowPassFilter(lowpassalpha, True)

    def filter_accel(self, gyro_x, gyro_y, gyro_z):
        gyro_x = self.gyro_x_filter.update(gyro_x)
        gyro_y = self.gyro_y_filter.update(gyro_y)
        gyro_z = self.gyro_z_filter.update(gyro_z)
        return gyro_x, gyro_y, gyro_z

    def filter_gyro(self, accel_x, accel_y, accel_z):
        accel_x = self.accel_x_filter.update(accel_x)
        accel_y = self.accel_y_filter.update(accel_y)
        accel_z = self.accel_z_filter.update(accel_z)
        return accel_x, accel_y, accel_z

    def read_accel(self, dt):
        while True:
            try:
                # accel_data = self.sensor.get_accel_data()
                # accel_x = accel_data['x']
                # accel_y = accel_data['y']
                # accel_z = accel_data['z']
                # # accel_x, accel_y, accel_z = self.filter_accel(accel_x, accel_y, accel_z)
                # return accel_x, accel_y, accel_z
                accel_data = self.sensor.get_accel_data()
                accel_x = accel_data[0]
                accel_y = accel_data[1]
                accel_z = accel_data[2]
                return accel_x, accel_y, accel_z
            except OSError as e:
                print("Error reading accelerometer data:", e)
                # time.sleep(dt)

    def read_gyro(self, dt):
        while True:
            try:
                # gyro_data = self.sensor.get_gyro_data()
                # gyro_x = gyro_data['x']
                # gyro_y = gyro_data['y']
                # gyro_z = gyro_data['z']
                # # gyro_x, gyro_y, gyro_z = self.filter_gyro(gyro_x, gyro_y, gyro_z)
                # gyro_z += 0.19
                # gyro_z = -gyro_z
                # gyro_x += 3.10
                # return gyro_x, gyro_y, gyro_z
                accel_data = self.sensor.get_gyro_data()
                gyro_x = accel_data[0]
                gyro_y = accel_data[1]
                gyro_z = accel_data[2]
                return gyro_x, gyro_y, gyro_z
            except OSError as e:
                print("Error reading gyroscope data:", e)
                # time.sleep(dt)


    def complementary_filter(self, gyro_data, accel_data, dt, roll, pitch, yaw):
        gyro_x, gyro_y, gyro_z = gyro_data
        accel_x, accel_y, accel_z = accel_data

        # 计算陀螺仪提供的姿态变化量
        roll_rate = gyro_x
        pitch_rate = gyro_y

        # 计算加速度计提供的姿态信息
        roll_acc = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi
        pitch_acc = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180 / math.pi

        # 使用互补滤波器结合陀螺仪和加速度计的信息
        roll = self.alpha * (roll + roll_rate * dt) + (1 - self.alpha) * roll_acc
        pitch = self.alpha * (pitch + pitch_rate * dt) + (1 - self.alpha) * pitch_acc

        # 更新yaw角度，直接使用陀螺仪提供的数据
        yaw = (yaw + gyro_z * dt * 1.66) % 360
        if yaw > 180:
            yaw -= 360

        return roll, pitch, yaw

    def calculate_euler_angles(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt, row, pitch, yaw):
        # 加速度计数据转换为重力加速度
        accel_x /= 16384.0
        accel_y /= 16384.0
        accel_z /= 16384.0

        row, pitch, yaw = self.complementary_filter((gyro_x, gyro_y, gyro_z), (accel_x, accel_y, accel_z), dt, row, pitch, yaw)

        return row, pitch, yaw
    
    def working_thread(self):
        while True:
            accel_x, accel_y, accel_z = self.read_accel(self.dt)
            gyro_x, gyro_y, gyro_z = self.read_gyro(self.dt)
            self.roll, self.pitch, self.yaw = self.calculate_euler_angles(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, self.dt, self.roll, self.pitch, self.yaw)
            # time.sleep(self.dt)

    def getdata(self):
        return [ float(self.roll), float(self.pitch), float(self.yaw) ]

    def run(self):
        t = threading.Thread(target=self.working_thread)
        t.start()


if __name__ == '__main__':
    import time

    # 记录第一次接收到数据的时间
    start_time = time.time()



    Posture = posture(0.01)
    Posture.run()
    import time
    import socket
    # 设置Socket连接
    HOST = '192.168.31.102'  # 主机IP地址
    PORT = 12345  # 端口号
    # 创建Socket对象
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind((HOST, PORT))
        s.listen(1)
        # 等待客户端连接
        print("Waiting for connection...")
        conn, addr = s.accept()
        print('Connected by', addr)
        while True:
            current_posture = Posture.getdata()
            data = f"{current_posture[0]},{current_posture[1]},{Posture.read_gyro(0)[0]},{Posture.read_gyro(0)[1]},{Posture.read_gyro(0)[2]},0, "
            # request = conn.recv(1024).decode('utf-8')
            # if request:
                # conn.sendall(data.encode('utf-8'))
            conn.sendall(data.encode('utf-8'))
            # data.clear()
            # time.sleep(0.01)
                # 计算时间间隔并打印
            end_time = time.time()
            time_interval = end_time - start_time
            print("Time interval since last 'Received':", time_interval, "seconds")

            # 更新起始时间
            start_time = end_time

    except  KeyboardInterrupt:
        s.close()
        print("Socket closed")
        sys.exit()

