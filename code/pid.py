# This is a PID controller class for controlling a SQ with feedback.

from motor import motor
from posture import posture
import time

import socket
import struct

class PIDController:
    def __init__(self, Kp, Ki, Kd, tolerance):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.tolerance = tolerance
        self.prev_error = 0
        self.integral = 0
        self.output = 0  # 新增输出值属性
        self.target_output = 0  # 新增目标输出值属性

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        self.output = max(-500, min(self.output, 500))

    def gradual_update(self, setpoint, measurement, dt, step_size):
        error = setpoint - measurement
        self.integral += error * dt
        if( abs(error) < self.tolerance ):
            error = 0
        derivative = (error - self.prev_error) / dt
        # derivative = abs(derivative)
        self.target_output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        if self.target_output > self.output:
            self.output = min(self.output + step_size, self.target_output)
        elif self.target_output < self.output:
            self.output = max(self.output - step_size, self.target_output)
        self.prev_error = error
        self.output = max(-500, min(self.output, 500))

def map_to_pwm(pid_output, pwm_min, pwm_max):
    pid_output = pid_output / 1000
    mid = 25
    if pid_output > 0 :
        pwm_output = pid_output * (pwm_max - mid) / 0.03 + mid
    else :
        pwm_output = pid_output * mid / 0.03 + mid
    return max(min(pwm_output, pwm_max), pwm_min)

# PID参数
Kp_roll = 0.3
Ki_roll = 0.02
Kd_roll = 0.04
Ktolerance_roll = 1

Kp_pitch = 0.01
Ki_pitch = 1
Kd_pitch = 50
Ktolerance_pitch = 0.5

# 最小和最大PWM占空比
pwm_min = 15
pwm_max = 30

# 创建PID控制器
roll_pid = PIDController(Kp_roll, Ki_roll, Kd_roll, Ktolerance_roll)
pitch_pid = PIDController(Kp_pitch, Ki_pitch, Kd_pitch, Ktolerance_pitch)

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
    time.sleep(dt)
    current_posture = Posture.getdata()

    # print(current_posture)

    current_roll = current_posture[0]
    current_pitch = current_posture[1]

    if abs(current_roll) > 70 or abs(current_pitch) > 30:
        global pwm_max
        global pwm_min
        pwm_max = 0
        pwm_min = 0
        for i in range(4):
            motor.update(i, 0)
            
        if abs(current_roll) > 70:
            print("roll Out of range!")
        if abs(current_pitch) > 25:
            print("pitch Out of range!")

    # print("Current Roll: ", current_roll)
    # print("Current Pitch: ", current_pitch)

    # # 使用PID控制器计算横滚和俯仰的输出
    # roll_output = roll_pid.update(desired_roll, -current_roll, dt)
    # pitch_output = pitch_pid.update(desired_pitch, current_pitch, dt)
    # 使用渐变控制更新PID输出
    step_size = 10  # 步长大小
    roll_pid.gradual_update(desired_roll, -current_roll, dt, step_size)
    pitch_pid.gradual_update(desired_pitch, current_pitch, dt, step_size)
    # roll_pid.update(desired_roll, -current_roll, dt)
    # pitch_pid.update(desired_pitch, current_pitch, dt)

    roll_output = roll_pid.output
    # print("Roll Output: ", roll_output)
    pitch_output = pitch_pid.output
    pitch_output = 0

    # 将输出映射为PWM占空比 正的都是要变大的项
    motor0_pwm = map_to_pwm(roll_output + pitch_output, pwm_min, pwm_max)
    motor1_pwm = map_to_pwm(-roll_output + pitch_output, pwm_min, pwm_max)
    motor2_pwm = map_to_pwm(-roll_output - pitch_output, pwm_min, pwm_max)
    motor3_pwm = map_to_pwm(roll_output - pitch_output, pwm_min, pwm_max)
    # print("rool_output: ", roll_output)
    # 将PWM占空比发送到四个电机
    # motor.update(motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm)
    # print(0, motor0_pwm)
    # print(3, motor3_pwm)
    motor.update( 0, motor0_pwm )
    motor.update( 3, motor3_pwm )
    motor.update( 1, motor1_pwm )
    motor.update( 2, motor2_pwm )
    # motor.update( 1, int(motor1_pwm * 0.75) )
    # motor.update( 2, int(motor2_pwm * 0.75) )

    # motor.update(0, 0)
    # motor.update(1, 20)
    # motor.update(2, 20)
    # motor.update(3, 0)

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

# 设置Socket连接
HOST = '192.168.31.68'  # 主机IP地址
PORT = 12312  # 端口号

# 创建Socket对象
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("Start Flying...")

import tty
import termios
import sys
import threading

def get_char():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def input_thread():
        global pwm_max, pwm_min
        while True :
            try:
                ch = get_char()
                print("you have pressed: ", ch)
                if ch == 'q' :
                    pwm_max = 0
                    pwm_min = 10
                    for i in range(4):
                        motor.update(i, 0)
                    # motor.stop()
                    # # exit()
                    # break
                if ch == 'w' :
                    pwm_max = pwm_max + 1
                if ch == 's' :
                    pwm_max = pwm_max - 1
                
                pwm_max = max(pwm_max, 0)
                pwm_max = min(pwm_max, 100)
                print("PWM_MAX: ", pwm_max)

            except:
                pass

threading.Thread(target=input_thread).start()
input_thread_obj = threading.Thread(target=input_thread)
input_thread_obj.start()

print("Connecting to server...")
s.connect((HOST, PORT))
print("Connected to server!")

print("START")

while True:
# for i in range(1):
    flight_loop(0.01)  # 采样时间间隔为 10ms

    def calSCAC(data):
        SC = 0x00
        AC = 0x00
        for i in range(len(data)):
            SC += data[i] & 0xFF
            AC += SC & 0xFF
            SC &= 0xFF
            AC &= 0xFF
        return SC, AC
    
    HEAD = 0xAA
    D_ADDR = 0xFF
    ID = 0x03
    LEN = 7
    current_posture = Posture.getdata()
    ROL, PIT, YAW = Posture.getdata()
    # print("ROL:", ROL)
    ROL *= 100
    ROL = int(ROL)
    ROL &= 0xFFFF
    PIT *= 100
    PIT = int(PIT)
    PIT &= 0xFFFF
    YAW *= 100
    YAW = int(YAW)
    YAW &= 0xFFFF
    YAW = 0
    FUSION_STA = 1
    DATA = [ROL & 0xFF, (ROL >> 8 ) & 0xFF, PIT & 0xFF, (PIT >> 8 ) & 0xFF, YAW & 0xFF, (YAW >> 8 ) & 0xFF, FUSION_STA]

    SC, AC = calSCAC([HEAD, D_ADDR, ID, LEN] + DATA)

    SC &= 0xFF
    AC &= 0xFF

    frame = bytes([HEAD, D_ADDR, ID, LEN, DATA[0], DATA[1], DATA[2], DATA[3], DATA[4], DATA[5], DATA[6], SC, AC])
    s.sendto(frame,(HOST, PORT))
    # print("ROLL", ROL)
    # print(f"Sending: {frame}")

    ID = 0x20
    LEN = 8
    MOTOR1 = int(motor.channelPulse[1] * 100)
    MOTOR2 = int(motor.channelPulse[2] * 100)
    MOTOR3 = int(motor.channelPulse[3] * 100)
    MOTOR4 = int(motor.channelPulse[0] * 100)
    DATA = [MOTOR1 & 0xFF, (MOTOR1 >> 8 ) & 0xFF, MOTOR2 & 0xFF, (MOTOR2 >> 8 ) & 0xFF, MOTOR3 & 0xFF, (MOTOR3 >> 8 ) & 0xFF, MOTOR4 & 0xFF, (MOTOR4 >> 8 ) & 0xFF]

    SC, AC = calSCAC([HEAD, D_ADDR, ID, LEN] + DATA)

    SC &= 0xFF
    AC &= 0xFF

    motorframe = bytes([HEAD, D_ADDR, ID, LEN, DATA[0], DATA[1], DATA[2], DATA[3], DATA[4], DATA[5], DATA[6], DATA[7], SC, AC])

    s.sendto(motorframe,(HOST, PORT))

    # print(f"Sending: {frame}")
    # time.sleep(2000)
    # time.sleep(0.01)
    continue
    

    current_posture = Posture.getdata()
    data = f"{current_posture[0]},{current_posture[1]},{motor.channelPulse[0]}, {motor.channelPulse[1]},{motor.channelPulse[2]},{motor.channelPulse[3]},{Posture.read_gyro(0)[0]},{Posture.read_gyro(0)[1]},{Posture.read_gyro(0)[2]},{Posture.read_accel(0)[0]},{Posture.read_accel(0)[1]},{Posture.read_accel(0)[2]},"
    request = conn.recv(1024).decode('utf-8')
    print("Prepared")
    if request:
        conn.sendall(data.encode('utf-8'))
        print("Send")
    # time.sleep(0.01)


# 关闭连接 
s.close()
