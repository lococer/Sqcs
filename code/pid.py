# This is a PID controller class for controlling a SQ with feedback.

from motor import motor
from posture import posture
import time

import socket
import struct

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

def map_to_pwm(pid_output, pwm_min, pwm_max):
    pwm_output = (pid_output + 1) / 2 * (pwm_max - pwm_min) + pwm_min
    return max(min(pwm_output, pwm_max), pwm_min)

# PID参数
Kp_roll = 1.0
Ki_roll = 0.1
Kd_roll = 0.2

Kp_pitch = 1.0
Ki_pitch = 0.1
Kd_pitch = 0.2

# 最小和最大PWM占空比
pwm_min = 1000
pwm_max = 2000

# 创建PID控制器
roll_pid = PIDController(Kp_roll, Ki_roll, Kd_roll)
pitch_pid = PIDController(Kp_pitch, Ki_pitch, Kd_pitch)

# 期望姿态
desired_roll = 0.0  # 期望横滚角
desired_pitch = 0.0  # 期望俯仰角

# 读取当前姿态
Posture = posture(0.01)
Posture.run()

motor = motor()
motor.run()

# 飞行循环
def flight_loop(dt):
    current_posture = Posture.getdata()

    # print(current_posture)

    current_roll = current_posture[0]
    current_pitch = current_posture[1]

    # print("Current Roll: ", current_roll)
    # print("Current Pitch: ", current_pitch)

    # 使用PID控制器计算横滚和俯仰的输出
    roll_output = roll_pid.update(desired_roll, -current_roll, dt)
    pitch_output = pitch_pid.update(desired_pitch, current_pitch, dt)

    # 将输出映射为PWM占空比 正的都是要变大的项
    motor0_pwm = map_to_pwm(roll_output + pitch_output, pwm_min, pwm_max)
    motor1_pwm = map_to_pwm(-roll_output + pitch_output, pwm_min, pwm_max)
    motor2_pwm = map_to_pwm(-roll_output - pitch_output, pwm_min, pwm_max)
    motor3_pwm = map_to_pwm(roll_output - pitch_output, pwm_min, pwm_max)

    # 将PWM占空比发送到四个电机
    # motor.update(motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm)
    motor.update( 0, motor0_pwm )
    motor.update( 1, motor1_pwm )
    motor.update( 2, motor2_pwm )
    motor.update( 3, motor3_pwm )

# 在飞行循环中调用 flight_loop(dt)，传入采样时间间隔 dt

def print_posture():
    current_posture = Posture.getdata()
    print("Current Roll: ", current_posture[0])
    print("Current Pitch: ", current_posture[1])

def print_motor_pwm():
    motor0_pwm = motor.channelPulse[0]
    motor1_pwm = motor.channelPulse[1]
    motor2_pwm = motor.channelPulse[2]
    motor3_pwm = motor.channelPulse[3]
    print("Motor 0 PWM: ", motor0_pwm)
    print("Motor 1 PWM: ", motor1_pwm)
    print("Motor 2 PWM: ", motor2_pwm)
    print("Motor 3 PWM: ", motor3_pwm)


cnt = 0

print("Start Flying...")

# 设置接收端的IP地址和端口号
UDP_IP = '192.168.31.68'
UDP_PORT = 25002

# 创建UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    while True:
        flight_loop(0.01)  # 采样时间间隔为 10ms
        current_posture = Posture.getdata()
        packed_data = struct.pack('ddiiii',current_posture[0],current_posture[1],motor.channelPulse[0],motor.channelPulse[1],motor.channelPulse[2],motor.channelPulse[3])
        sock.sendto(packed_data, (UDP_IP, UDP_PORT))
        time.sleep(0.01)
        cnt += 1
        if cnt % 100 == 0:
            # print_posture()
            print("Current Roll: ", current_posture[0])
            print("Current Pitch: ", current_posture[1])
            print_motor_pwm()

except KeyboardInterrupt:
    print("Closing connection...")
    # 关闭socket连接
    sock.close()