import socket
import struct
import time
import sys
sys.path.append("/home/orangepi/.local/lib/python3.10/site-packages")
from mpu6050 import mpu6050

# 定义要发送的单精度数据
double_data = 3.14159265359

# 将单精度数据转换为字节流
packed_data = struct.pack('d', double_data)

# 设置接收端的IP地址和端口号
UDP_IP = '192.168.31.68'
UDP_PORT = 25000

# 创建UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 发送数据
try:
    # times = 0
    while True:

        # 读取MPU6050传感器的数据
        sensor = mpu6050(0x68)
        accel_data = sensor.get_accel_data()
        accel_data['x'] = accel_data['x'] + 0.532
        accel_data['y'] = accel_data['y'] + 0.2283
        # accel_data['z'] = accel_data['z'] - 0.7603
        gyro_data = sensor.get_gyro_data()
        temp_data = sensor.get_temp()
        
        #打包数据
        packed_data = struct.pack('ddd', accel_data['x'], accel_data['y'], accel_data['z'])
        packed_data += struct.pack('ddd', gyro_data['x'], gyro_data['y'], gyro_data['z'])
        # packed_data += struct.pack('d', temp_data)

        # 发送数据
        sock.sendto(packed_data, (UDP_IP, UDP_PORT))


        # time.sleep(0.01)
        # times += 1
        # double_data = times % 20 - 10.0
        # packed_data = struct.pack('d', double_data)
        # # print(times, "Sending data: ", double_data)
        # sock.sendto(packed_data, (UDP_IP, UDP_PORT))

except KeyboardInterrupt:
    print("Closing connection...")
    # 关闭socket连接
    sock.close()