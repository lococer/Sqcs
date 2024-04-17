import socket
import sys
sys.path.append("/home/orangepi/.local/lib/python3.10/site-packages")
import math
import time
from mpu6050 import mpu6050

# 初始化MPU6050
sensor = mpu6050(0x68)

# 创建socket连接
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('192.168.31.102', 8000))  # 绑定树莓派IP和端口
server_socket.listen(1)

print("Waiting for connection...")
client_socket, client_address = server_socket.accept()
print(f"Connection from: {client_address}")

# 初始化互补滤波参数
alpha = 0.98  # 互补滤波系数
dt = 0.01  # 时间间隔，假设为0.01秒

try:
    while True:
        data = client_socket.recv(1024).decode().strip()  # 接收数据并解码
        if data != "get_data": continue  # 如果不是get_data命令，则跳过

        accel_data = sensor.get_accel_data()
        gyro_data = sensor.get_gyro_data()

        # 计算加速度计测得的倾斜角度
        roll_acc = math.atan2(accel_data['y'], accel_data['z'])
        roll_acc = math.degrees(roll_acc)  # 将弧度转换为角度
        pitch_acc = math.atan2(-accel_data['x'], math.sqrt(accel_data['y'] ** 2 + accel_data['z'] ** 2))
        pitch_acc = math.degrees(pitch_acc)  # 将弧度转换为角度

        # 计算陀螺仪测得的角速度（使用角度作为单位）
        roll_gyro = math.degrees(gyro_data['x'])
        pitch_gyro = math.degrees(gyro_data['y'])

        # 互补滤波融合加速度计和陀螺仪数据
        roll = alpha * (roll_gyro + dt * roll_acc) + (1 - alpha) * roll_acc
        pitch = alpha * (pitch_gyro + dt * pitch_acc) + (1 - alpha) * pitch_acc


        # 将滤波后的角度数据转换为字符串
        data_str = f"Roll: {math.degrees(roll)} Pitch: {math.degrees(pitch)}\n"

        # 发送数据到本地
        client_socket.send(data_str.encode())
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Closing connection...")
    client_socket.close()
    server_socket.close()
