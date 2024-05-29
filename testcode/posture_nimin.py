import socket
from mpu6050_kalman import GyroKalmanFilter

# This code is used to estimate the posture of the robot using the MPU6050 sensor.
# using complementary filter to combine the information from the gyroscope and accelerometer.
# using getdata() function to get the current posture of the robot.

import time
import sys
sys.path.append("/home/orangepi/.local/lib/python3.10/site-packages")
# from mpu6050 import mpu6050
import math
import threading
import struct
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
                # Aaccel_x = accel_data['x']
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

Posture = posture(0.01)
Posture.run()

# 函数用于获取数据
def getdata():
    # 获取姿态数据
    roll, pitch, yaw = Posture.getdata()[:3]
    # 将姿态数据乘以 100 并转换为整数，确保在 0 到 255 范围内
    roll_int = int(roll * 100)
    pitch_int = int(pitch * 100)
    yaw_int = int(yaw * 100)
    # 将数据限制在 0 到 255 范围内
    # roll_int = max(min(roll_int, 255), 0)
    # pitch_int = max(min(pitch_int, 255), 0)
    # yaw_int = max(min(yaw_int, 255), 0)

    # print("roll_int:", roll_int, "pitch_int:", pitch_int, "yaw_int:", yaw_int)
    return roll_int & 0xFFFF, pitch_int & 0xFFFF, yaw_int & 0xFFFF, 1


# 设置目标地址和端口
HOST = '192.168.31.68'  # 目标地址
PORT = 12312  # 目标端口

# 创建socket对象
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#try:
    # 连接到目标地址和端口
print("Connecting to server...")
s.connect((HOST, PORT))
print("Connected to server!")

def calSCAC(data):
    SC = 0x00
    AC = 0x00
    for i in range(len(data)):
        SC += data[i] & 0xFF
        AC += SC & 0xFF
        SC &= 0xFF
        AC &= 0xFF
    return SC, AC

while True:
    # 获取数据 横滚 俯仰 偏航 融合状态
    ROL, PIT, YAW, FUSION_STA = getdata()
    YAW = 0
    print("ROL:", ROL, "PIT:", PIT, "YAW:", YAW, "FUSION_STA:", FUSION_STA)
    # ROL, PIT, YAW, FUSION_STA = 0, 0, 0, 1
    # print("ROL:", ROL, "PIT:", PIT, "YAW:", YAW, "FUSION_STA:", FUSION_STA)
    
    HEAD = 0xAA
    D_ADDR = 0xFF
    ID = 0x03
    LEN = 7
    DATA = [ROL & 0xFF, (ROL >> 8 ) & 0xFF, PIT & 0xFF, (PIT >> 8 ) & 0xFF, YAW & 0xFF, (YAW >> 8 ) & 0xFF, FUSION_STA]
    SC, AC = calSCAC([HEAD, D_ADDR, ID, LEN, DATA[0], DATA[1], DATA[2], DATA[3],DATA[4], DATA[5], DATA[6]])

    SC &= 0xFF
    AC &= 0xFF

    # 构造帧
    #frame = bytearray([HEAD >> 8, HEAD & 0xFF, D_ADDR, ID, LEN])
    # print(DATA[0], DATA[1], DATA[2], DATA[3], SC, AC)
    frame = bytes([HEAD, D_ADDR, ID, LEN, DATA[0], DATA[1], DATA[2], DATA[3],DATA[4], DATA[5], DATA[6], SC, AC])
    
    # print("Frame:", frame)
    # 发送帧
    s.sendto(frame, (HOST, PORT))
    # print("Frame sent:", frame)
    
    # print("Frame sent successfully!")
    time.sleep(0.01)
s.close()

#except Exception as e:
    #print("Error:", e)
#finally:
    # 关闭socket连接
    #s.close()
