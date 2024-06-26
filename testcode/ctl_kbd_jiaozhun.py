# This code is used to control motors using keyboard.

import pca9685
from pca9685 import get_char
import threading
import time
import smbus

class ctl_kbd:
    def __init__(self, debug=True):
        # special initialization for PCA9685 module
        pwm = pca9685.PCA9685(0x40, debug=True)
        pwm.setPWMFreq(50)
        for i in range(700,800,10):  
            pwm.setServoPulse(0,i)   
            pwm.setServoPulse(1,i) 
            pwm.setServoPulse(2,i)   
            pwm.setServoPulse(3,i) 
        del pwm
        # End of special initialization for PCA9685 module
        self.pwm = pca9685.PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)
        self.channelPulse = [0,0,0,0]
        self.stopped = True
        self.debug = debug
        print("finished initialization")

    def __del__(self):
        self.pwm.setAllPulse(1000)
        print("Cleaning up")

    def work_thread(self):
        while self.stopped:
            for i, v in enumerate(self.channelPulse):
                v = (int)(0.01 * (2000 - 1000) * v + 1000 )
                self.pwm.setServoPulse(i, v)

    def input_thread(self):
        channel = 4
        while self.stopped:
            try:
                ch = get_char()
                if self.debug:
                    print("you have pressed: ", ch)
                if ch == 'q' :
                    self.stopped = False
                if ch == 'w' :
                    self.channelPulse[channel] += 1
                    self.channelPulse[channel] = min(self.channelPulse[channel], 100)
                if ch == 's' :
                    self.channelPulse[channel] -= 1
                    self.channelPulse[channel] = max(self.channelPulse[channel], 0)
                if ch == 'd' :
                    self.channelPulse[channel] = 0

                if ch == 'D':
                    for i in range(4):
                        self.channelPulse[i] = 0
                if ch == 'W':
                    for i in range(4):
                        self.channelPulse[i] += 1
                        self.channelPulse[i] = min(self.channelPulse[i], 100)
                if ch == 'S':
                    for i in range(4):
                        self.channelPulse[i] -= 1
                        self.channelPulse[i] = max(self.channelPulse[i], 0)

                if ch == '0' :
                    channel = 0
                if ch == '1' :
                    channel = 1
                if ch == '2' :
                    channel = 2
                if ch == '3' :
                    channel = 3
                
                
                for i, v in enumerate(self.channelPulse):
                    print("chanel: ", i, " efficiency: ", v)

            except:
                pass

    def run(self):
        input_thread_obj = threading.Thread(target=self.input_thread)
        input_thread_obj.start()
        work_thread_obj = threading.Thread(target=self.work_thread)
        work_thread_obj.start()
        input_thread_obj.join()
        work_thread_obj.join()

## Unknow reason, this function is necessary to initialize the PCA9685 module
## furthur investigation: when ceate PCA9685 object, you need to set debug = True
def special_ini():
    pwm = pca9685.PCA9685(0x40, debug=True)
    pwm.setPWMFreq(50)

    for i in range(700,800,10):  
        pwm.setServoPulse(0,i)   
        pwm.setServoPulse(1,i) 
        pwm.setServoPulse(2,i)   
        pwm.setServoPulse(3,i) 
        # time.sleep(0.1)     
        print(i)
    
    del pwm


def main():
    # Initializing the PCA9685 module
    special_ini()
    demo = ctl_kbd(True)
    demo.run()
    del demo

main()
    
