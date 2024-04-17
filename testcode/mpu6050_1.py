import time
import sys
sys.path.append("/home/orangepi/.local/lib/python3.10/site-packages")
from mpu6050 import mpu6050
import math
import socket
import struct

UDP_IP = '192.168.31.68'
UDP_PORT = 25001
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.filtered_value = None

    def update(self, new_value):
        if self.filtered_value is None:
            self.filtered_value = new_value
        else:
            self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value

gyro_x_filter = LowPassFilter(0.1)
gyro_y_filter = LowPassFilter(0.1)
gyro_z_filter = LowPassFilter(0.1)

accel_x_filter = LowPassFilter(0.1)
accel_y_filter = LowPassFilter(0.1)
accel_z_filter = LowPassFilter(0.1)


def filter_accel(gyro_x, gyro_y, gyro_z):
    gyro_x = gyro_x_filter.update(gyro_x)
    gyro_y = gyro_y_filter.update(gyro_y)
    gyro_z = gyro_z_filter.update(gyro_z)
    return gyro_x, gyro_y, gyro_z

def filter_gyro(accel_x, accel_y, accel_z):
    accel_x = accel_x_filter.update(accel_x)
    accel_y = accel_y_filter.update(accel_y)
    accel_z = accel_z_filter.update(accel_z)
    return accel_x, accel_y, accel_z

sensor = mpu6050(0x68)

def read_accel(dt):
    while True:
        try:
            accel_data = sensor.get_accel_data()
            accel_x = accel_data['x']
            accel_y = accel_data['y']
            accel_z = accel_data['z']
            accel_x, accel_y, accel_z = filter_accel(accel_x, accel_y, accel_z)
            return accel_x, accel_y, accel_z
        except OSError as e:
            print("Error reading accelerometer data:", e)
            time.sleep(dt)  # 等待一段时间后重试

def read_gyro(dt):
    while True:
        try:
            gyro_data = sensor.get_gyro_data()
            gyro_x = gyro_data['x']
            gyro_y = gyro_data['y']
            gyro_z = gyro_data['z']
            gyro_x, gyro_y, gyro_z = filter_gyro(gyro_x, gyro_y, gyro_z)
            gyro_z += 0.19
            gyro_z = -gyro_z
            return gyro_x, gyro_y, gyro_z
        except OSError as e:
            print("Error reading gyroscope data:", e)
            time.sleep(dt)  # 等待一段时间后重试

alpha = 0.98  # 陀螺仪权重，加速度计权重为 1 - alpha

def complementary_filter(gyro_data, accel_data, dt, roll, pitch, yaw):
    gyro_x, gyro_y, gyro_z = gyro_data
    accel_x, accel_y, accel_z = accel_data

    # 计算陀螺仪提供的姿态变化量
    roll_rate = gyro_x
    pitch_rate = gyro_y

    # 计算加速度计提供的姿态信息
    roll_acc = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi
    pitch_acc = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180 / math.pi

    # 使用互补滤波器结合陀螺仪和加速度计的信息
    roll = alpha * (roll + roll_rate * dt) + (1 - alpha) * roll_acc
    pitch = alpha * (pitch + pitch_rate * dt) + (1 - alpha) * pitch_acc

    # 更新yaw角度，直接使用陀螺仪提供的数据
    yaw = (yaw + gyro_z * dt * 1.66) % 360
    if yaw > 180:
        yaw -= 360

    return roll, pitch, yaw

def calculate_euler_angles(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt, row, pitch, yaw):
    # 加速度计数据转换为重力加速度
    accel_x /= 16384.0
    accel_y /= 16384.0
    accel_z /= 16384.0

    row, pitch, yaw = complementary_filter((gyro_x, gyro_y, gyro_z), (accel_x, accel_y, accel_z), dt, row, pitch, yaw)

    return row, pitch, yaw

def main():
    dt = 0.01 # 采样时间间隔（秒）
    yaw = 0 # 初始偏航角为0度
    roll = 0 # 初始滚动角为0度
    pitch = 0 # 初始俯仰角为0度
    print("开始姿态估计")
    while True:
        accel_x, accel_y, accel_z = read_accel(dt)
        gyro_x, gyro_y, gyro_z = read_gyro(dt)
        roll, pitch, yaw = calculate_euler_angles(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt, roll, pitch, yaw)
        
        packed_data = struct.pack('fff', roll, pitch, yaw)
        # packed_data = struct.pack('fff', pitch, gyro_z, yaw)
        sock.sendto(packed_data, (UDP_IP, UDP_PORT))

        # print("Roll:", roll, "Pitch:", pitch, "Yaw:", yaw)
        # print(gyro_z)
        
        time.sleep(dt)

if __name__ == "__main__":
    main()
