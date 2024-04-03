#!/usr/bin/python

import time
import math
import smbus
import threading
import sys
import tty
import termios

# ============================================================================
# Raspi PCA9685 16-Channel PWM Servo Driver
# ============================================================================

# 控制电机
class PCA9685:

	# Registers/etc.
	__SUBADR1            = 0x02
	__SUBADR2            = 0x03
	__SUBADR3            = 0x04
	__MODE1              = 0x00
	__PRESCALE           = 0xFE
	__LED0_ON_L          = 0x06
	__LED0_ON_H          = 0x07
	__LED0_OFF_L         = 0x08
	__LED0_OFF_H         = 0x09
	__ALLLED_ON_L        = 0xFA
	__ALLLED_ON_H        = 0xFB
	__ALLLED_OFF_L       = 0xFC
	__ALLLED_OFF_H       = 0xFD

	def __init__(self, address=0x40, debug=False):
		self.bus = smbus.SMBus(3)
		self.address = address
		self.debug = debug
		if (self.debug):
			print("Reseting PCA9685")
			self.write(self.__MODE1, 0x00)

	def write(self, reg, value):
		"Writes an 8-bit value to the specified register/address"
		self.bus.write_byte_data(self.address, reg, value)
		if (self.debug):
			print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))

	def read(self, reg):
		"Read an unsigned byte from the I2C device"
		result = self.bus.read_byte_data(self.address, reg)
		if (self.debug):
			print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
		return result

	def setPWMFreq(self, freq):
		"Sets the PWM frequency"
		prescaleval = 25000000.0    # 25MHz
		prescaleval /= 4096.0       # 12-bit
		prescaleval /= float(freq)
		prescaleval -= 1.0
		if (self.debug):
			print("Setting PWM frequency to %d Hz" % freq)
			print("Estimated pre-scale: %d" % prescaleval)
		prescale = math.floor(prescaleval + 0.5)
		if (self.debug):
			print("Final pre-scale: %d" % prescale)

		oldmode = self.read(self.__MODE1);
		newmode = (oldmode & 0x7F) | 0x10        # sleep
		self.write(self.__MODE1, newmode)        # go to sleep
		self.write(self.__PRESCALE, int(math.floor(prescale)))
		self.write(self.__MODE1, oldmode)
		time.sleep(0.005)
		self.write(self.__MODE1, oldmode | 0x80)

	def setPWM(self, channel, on, off):	
		"Sets a single PWM channel"
		on = (int)(on)
		off = (int)(off)
		self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
		self.write(self.__LED0_ON_H+4*channel, on >> 8)
		self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
		self.write(self.__LED0_OFF_H+4*channel, off >> 8)
		if (self.debug):
			print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel,on,off))

	def setServoPulse(self, channel, pulse):
		"Sets the Servo Pulse,The PWM frequency must be 50HZ"
		pulse = pulse*4096/20000        #PWM frequency is 50HZ,the period is 20000us
		self.setPWM(channel, 0, pulse)
		# self.setPWM(channel, pulse, 0 )

	def setAllPWM(self,on, off):
		"Sets all PWM channel"
		on = int(on)
		off = int(off)
		self.write(self.__ALLLED_ON_L, on & 0xFF)
		self.write(self.__ALLLED_ON_H, on >> 8)
		self.write(self.__ALLLED_OFF_L, off & 0xFF)
		self.write(self.__ALLLED_OFF_H, off >> 8)
		if (self.debug):
			# print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel,on,off))
			print("setAllPWM: ON %d OFF %d" % (on,off))
	
	def setAllPulse(self, pulse):
		pulse = pulse*4096/20000
		self.setAllPWM(0, pulse)

pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)

def get_char():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

stopped = True
channel = 3
pulse = 0
channelPulse = [0,0,0,0]

def work():
	global stopped, channelPulse

	while stopped:
		for i, v in enumerate(channelPulse):
			v = int(v * 0.01 * 851 + 724)
			pwm.setServoPulse(i, v)

def input_thread():
	global stopped, channelPulse
	value = 0
	channel = 4
	while stopped:
		try:
			ch = get_char()
			print("you have pressed: ", ch)
			if ch == 'q' :
				stopped = False
			if ch == 'w' :
				channelPulse[channel] += 1
				channelPulse[channel] = min(channelPulse[channel], 100)
			if ch == 's' :
				channelPulse[channel] -= 1
				channelPulse[channel] = max(channelPulse[channel], 0)
			if ch == 'd' :
				channelPulse[channel] = 0
			if ch == '0' :
				channel = 0
			if ch == '1' :
				channel = 1
			
			for i, v in enumerate(channelPulse):
				print("chanel: ", i, " efficiency: ", v)

		except:
			pass

def ini():
    # 初始状态
    pwm.setAllPulse(700)

def end():
    # 结束状态
    pwm.setAllPulse(700)

if __name__ == '__main__':
    ini()

    # 创建一个线程用于控制输入
    input_thread_obj = threading.Thread(target=input_thread)
    input_thread_obj.start()

    task_thread = threading.Thread(target=work)
    task_thread.start()

    input_thread_obj.join()
    task_thread.join()

    end()