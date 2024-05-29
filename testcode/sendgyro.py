import sys
sys.path.append("/home/orangepi/.local/lib/python3.10/site-packages")
from mpu6050 import mpu6050
import socket

# 创建UDP套接字
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 绑定要监听的IP地址和端口
listen_addr = ('', 12300)  # 在所有网络接口上监听，端口号为12345
udp_socket.bind(listen_addr)


if __name__ == '__main__':
    # 初始化MPU6050传感器
    sensor = mpu6050(0x68)

    # while True:
    for i in range(10):
        accel_data = sensor.get_accel_data()
        gyro_data = sensor.get_gyro_data()
        print("Accelerometer data: x: %.2f, y: %.2f, z: %.2f" % (accel_data['x'], accel_data['y'], accel_data['z']))
        print("Gyroscope data: x: %.2f, y: %.2f, z: %.2f" % (gyro_data['x'], gyro_data['y'], gyro_data['z']))



