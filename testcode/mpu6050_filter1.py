import socket
import sys
sys.path.append("/home/orangepi/.local/lib/python3.10/site-packages")

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

accel_data = {'x': 0, 'y': 0, 'z': 0, 'x_prev': 0, 'y_prev': 0, 'z_prev': 0, 'x_prev_prev': 0, 'y_prev_prev': 0, 'z_prev_prev': 0}
gyro_data = {'x': 0, 'y': 0, 'z': 0, 'x_prev': 0, 'y_prev': 0, 'z_prev': 0, 'x_prev_prev': 0, 'y_prev_prev': 0, 'z_prev_prev': 0}

try:
    while True:
        data = client_socket.recv(1024).decode().strip()  # 接收数据并解码
        if data != "get_data": continue  # 如果不是get_data命令，则跳过
        # 读取加速度计和陀螺仪数据
        accel_data_new = sensor.get_accel_data()
        gyro_data_new = sensor.get_gyro_data()

        # 对x,y,z三个方向滤波
        accel_data['x'] = (accel_data_new['x'] + accel_data['x'] + accel_data['x_prev']) / 3
        accel_data['y'] = (accel_data_new['y'] + accel_data['y'] + accel_data['y_prev']) / 3
        accel_data['z'] = (accel_data_new['z'] + accel_data['z'] + accel_data['z_prev']) / 3
        gyro_data['x'] = (gyro_data_new['x'] + gyro_data['x'] + gyro_data['x_prev']) / 3
        gyro_data['y'] = (gyro_data_new['y'] + gyro_data['y'] + gyro_data['y_prev']) / 3
        gyro_data['z'] = (gyro_data_new['z'] + gyro_data['z'] + gyro_data['z_prev']) / 3

        # 更新前几次数据
        accel_data['x_prev_prev'] = accel_data['x_prev']
        accel_data['y_prev_prev'] = accel_data['y_prev']
        accel_data['z_prev_prev'] = accel_data['z_prev']
        gyro_data['x_prev_prev'] = gyro_data['x_prev']
        gyro_data['y_prev_prev'] = gyro_data['y_prev']
        gyro_data['z_prev_prev'] = gyro_data['z_prev']
        accel_data['x_prev'] = accel_data['x']
        accel_data['y_prev'] = accel_data['y']
        accel_data['z_prev'] = accel_data['z']
        gyro_data['x_prev'] = gyro_data['x']
        gyro_data['y_prev'] = gyro_data['y']
        gyro_data['z_prev'] = gyro_data['z']    

        # 将数据转换为字符串
        data_str = f"Accelerometer: {accel_data['x']},{accel_data['y']},{accel_data['z']} " \
                   f"Gyroscope: {gyro_data['x']},{gyro_data['y']},{gyro_data['z']}\n"

        # 发送数据到本地
        client_socket.send(data_str.encode())

except KeyboardInterrupt:
    print("Closing connection...")
    client_socket.close()
    server_socket.close()
