from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from TMotorCANControl.mit_can import TMotorManager_mit_can

# CHANGE THESE TO MATCH YOUR DEVICE!
Type_1 = 'AK60-6'
##########################################################
# Motor CAN addresses

ID_1 = 3   # left lower_leg
ID_2 = 2  # left upper_leg
ID_3 = 6    #left pitch
ID_4 = 8    #left jaw

ID_5 = 5    # Right lower_leg
ID_6 = 4  # right upper_leg
ID_7 = 7  #right pitch
ID_8 = 9   #right jaw


ID_9 = 10  # neck
ID_10 = 11  #right anckle
ID_11 = 12  #left anckle



motors = [] 

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


         
def read_variables():

    global motor_currents
    global variables_dict

    
    for idx, motor in enumerate(motors):
        #    motor.update
        motors[idx].update()
        time.sleep(0.5)
        
    total_current = 0.0    
    # Initialize an empty dictionary
    angles_list = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"]
    currents_list = ["I m3", "I m2", "I m6", "I m8", "I m5", "I m4", "I m7", "I m9", "I m10", "I m11", "I m12", "I tot"]
    torques_list = ["Torque3", "Torque2", "Torque6", "Torque8", "Torque5", "Torque4", "Torque7", "Torque9", "Torque10", "Torque11", "Torque12"]
    variables_dict = {}
    
    for idx, motor in enumerate(motors):
 

        theta = motor.position
        current = abs(motor._motor_state.current)
        torque = motor.torque
        total_current += current

            #Create a dictionary for publish values
        variables_dict[angles_list[idx]] = round(np.degrees(theta),2)
        variables_dict[currents_list[idx]] = round(current,2)
        variables_dict[torques_list[idx]] = round(torque,2)


        
    variables_dict[ "I tot"] = round(total_current,2)   
    
  

    
    
    #publish_variables()

    print("\r  ============================================================")    
    print("\r  variables_dict :", variables_dict)

    print("\r  ============================================================")       
    print("\r  ============================================================")
    print("\n  ->->->->->-> total_current ->->->->-> ", total_current)  
    print("\r  ============================================================")   
 
  

if __name__ == '__main__':
    with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_1) as dev1:
        with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_2) as dev2:
            with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_3) as dev3:
                with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_4) as dev4:
                        with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_5) as dev5:
                            with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_6) as dev6:
                                with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_7) as dev7:
                                    with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_8) as dev8:
                                         with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_9) as dev9:
                                             with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_10) as dev10:
                                                with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_11) as dev11: 
                                                    #Add motor objects to a list called motors
                                                    motors.extend([dev1,dev2,dev3,dev4,dev5,dev6,dev7,dev8,dev9,dev10,dev11])
                                                    read_variables()