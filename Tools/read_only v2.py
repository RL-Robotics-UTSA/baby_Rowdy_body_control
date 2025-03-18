from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK60-6'
ID = 11



def read_only(dev):
    dev.set_zero_position() # has a delay!
    time.sleep(1.5) # wait for the motor to zero (~1 second)
    
    print("Starting read only demo. Press ctrl+C to quit.")
    loop = SoftRealtimeLoop(dt=0.1, report=True, fade=0.0)
    
    for t in loop:
        print(" temp =", dev.get_temperature_celsius())
        dev.update()
        print(" theta1 =", dev.position)
        time.sleep(0.5)
        print(" temp =", dev.get_temperature_celsius())


if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        read_only(dev)