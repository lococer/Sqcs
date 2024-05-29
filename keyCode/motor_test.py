import threading
import termios
import sys
import tty
from motor import motor

# 创建电机控制类的实例
motor_control = motor(debug=False)

# 定义停止所有电机的函数
def stop_motors():
    for channel in range(4):
        motor_control.update(channel, 0)
    motor_control.stop()

# 定义更改PWM并更新电机的函数
def change_pwm(channel, direction):
    # 假设当前PWM值为0，可以根据实际情况调整
    current_pwm = motor_control.channelPulse[channel]
    if direction == 'increase':
        new_pwm = min(current_pwm + 10, 800)  # 增加PWM并确保不超过800
    elif direction == 'decrease':
        new_pwm = max(current_pwm - 10, 0)    # 减少PWM并确保不低于0
    motor_control.update(channel, new_pwm)

def get_char():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# 定义键盘输入监听函数
def keyboard_control():
    try:
        # 保存原始的tty设置
        fd = sys.stdin.fileno()
        original_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)  # 设置为无缓冲模式

        print("使用键盘控制电机，输入 'q' 退出程序。")
        while True:
            input_char = get_char()
            if input_char == 'w':
                print("增加电机的PWM")
                change_pwm(0, 'increase')
                change_pwm(1, 'increase')
                change_pwm(2, 'increase')
                change_pwm(3, 'increase')
            elif input_char == 's':
                print("减少电机的PWM")
                change_pwm(0, 'decrease')
                change_pwm(1, 'decrease')
                change_pwm(2, 'decrease')
                change_pwm(3, 'decrease')
            elif input_char == 'q':
                motor_control.stop()
                break
            # 可以添加更多的条件分支来控制不同的电机
    except KeyboardInterrupt:
        print("\n程序被用户中断。")
    finally:
        # 恢复原始的tty设置
        termios.tcsetattr(fd, termios.TCSADRAIN, original_settings)
        stop_motors()  # 停止所有电机

# 启动电机控制线程
motor_control.run()

# 开始键盘控制
keyboard_control()