from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# CHANGE THESE TO MATCH YOUR DEVICE!
Type = 'AK60-6'
ID = 3
range = 0.4

def position_step(dev1):
    dev1.set_zero_position() # has a delay!
    time.sleep(1.5)
    dev1.set_impedance_gains_real_unit(K=10,B=0.5)
    
    offset1 = 0.0
    delta1 = offset1/100.0
    pos_actual1 = 0.0
    print("Starting position step demo. Press ctrl+C to quit.")

    loop = SoftRealtimeLoop(dt = 0.02, report=True, fade=0)
    for t in loop:
        dev1.update()
        if t < 1.0:
            pos_actual1 = pos_actual1 + delta1
            dev1.position = pos_actual1
        else:
            #dev1.set_impedance_gains_real_unit(K=5,B=0.5)
            dev1.position = offset1 + range*np.sin(np.pi*t)
            # Estos prints aqui producen una falla en el update del motor despues de unos segundos!
            
            #print("\r torque1 =", dev1.torque)
            #print(" theta1 =", dev1.position)
            #print(" velocity1 =", dev1.velocity)
            print(" Current1 =", dev1._motor_state.current)
            #print(" Temperature1 =", dev1._motor_state.temperature)
            


    del loop

if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev1:
        position_step(dev1)