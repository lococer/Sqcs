# this code is used to controll the motor by updating the pulse width of each channel.
# using motor.run() to start the motor.
# using motor.update(channel, pulse) to update the pulse width of each channel.

import pca9685
import threading

class motor:
    lock = threading.Lock()
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
        self.channelmap = { 0 : 1, 1 : 2, 2 : 3, 3 : 0 }
        print("finished initialization")

    def __del__(self):
        self.pwm.setAllPulse(1000)
        print("Cleaning up")

    def update(self, channel, pulse):
        with self.lock:
            channel = self.channelmap[channel]
            self.channelPulse[channel] = (pulse)

    def work_thread(self):
        while self.stopped:
            for i, v in enumerate(self.channelPulse):
                v = (int)(0.01 * (2000 - 1000) * v + 1000 )
                self.pwm.setServoPulse(i, v)
        for i in range(5):
            self.pwm.setServoPulse(i, 1000)
        return


    def run(self):
        work_thread_obj = threading.Thread(target=self.work_thread)
        work_thread_obj.start()

    def stop(self):
        self.stopped = False
