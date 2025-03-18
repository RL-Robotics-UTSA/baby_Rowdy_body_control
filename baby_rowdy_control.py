
'''

#UTSA
#Sergio Montufar Research Project
Spring 2024
v1.0
March 12 2024

v2.0
March 15 2024
Use of a global array for the motors
Use of arrays for the positions
v3.0
March 16 2024
incorporate MQTT
incorporate the set_ang_position(position) function. It goes directly to that position in cartesian plane 
v4.0
March 22 2024
Introduce the trajectory_XYZ function, to go to the desired position in XYZ through a 100 points trajectory in cartesian plane
Introduce the trajectory_ang function, to go to the desired position in XYZ through a 100 points trajectory in angular space
Introduce the uo_down function, using 100 point of angular increments
v6.0
March 23 2024
This program assumes Initial position VERTICAL!!
INITIAL POSITION of the robot MUST be Completely vertical!! (Requires the support)
    - adjust forward_kinematics and inverse_kinematics to acomplish that
    - The outputs of these routines are then modified to be consistent with the robot motors. this was tested using  debug =1 in each function under test
    
Add debug variable to control how many debug prints are executed. 
    debug vaiable is set just before each call to main functions in the main control routine
    - debug = 0 removes all prints and allow sending movements to the motors
    - debug = 1 prints seversl messages but does NOT send commands to the motors
 
 Working functions:
    - Home
    - reset
    - up_down   
    - TrajectoryXYZ   For now it uses an interpolation in the ANGULAR SPACE, NOT in the Cartesian space

v7.0
March 24 2024
Correct return to home function
Correct forward_kinematics and inverse_kinematic fuctions 
INITIAL POSITION of the robot MUST be Completely vertical!! (Requires the support)
    - forward_kinematics and inverse_kinematics to acomplish that
    
 Working functions:
    - Home
    - reset
    - sinusoidal_up_down   
    - TrajectoryXYZ   For now it uses an interpolation in the ANGULAR SPACE, NOT in the Cartesian space
    - sinusoidal step
v8.0
April 5 2024
 New Working functions:
    - sinusoidal side_step
    - display_ang_currents
    - display_ang_torques
    
    Publish angles on topic body/monitor/angles
    Publish torques on topic body/monitor/torques
    Publish currents on topic body/monitor/currents
    
v9.0
April 6 2024    
First test in floor
    - Increase the PD  P gin a lot!
    - Include OVER current protection MAX_CURRENT = 14
    - It walks backwards!
    - Modified the walker support to let the robot go up-down freely
    - Modify User interface to siplay aal the curents
    
v10.0
April 12 2024  
Publish total current
publish all the variables every 500 mSeg
  
  
  
v11.0
April 14-21 2024  
Modify the main program to be a low level control:
    - every 10 mSeg is reading variables and sending positions to the motors
    - every 100 mSeg is publishing variables
    - the motor positions come from upper level equilibrium control or
    - the motor positions come from upper level trayectory control
 Working functions with the new low level control (every 10 mSecs) :
    - Trajectory_XYZ
    - Home
    - Reset
    - up_down
    
    
v12.0
April 21 2024  
 Working functions with the new low level control (every 10 mSecs) :
    - sinusolidal_step
    - sinusolidal_side_step

v13.0
April 23 2024 
    - This is the first time it stands at home  stable, consuming only 4 Amps total!
    
    
v14.0
April 24 2024 

v15.0
April 30 2024 
    - modification to step routine with 3 different sinusoidals with a delay between them
    May 7 2024
    -walk parameters now can be adjusted from the GUI
    
v16.0
May 9 2024 
    - home parameters now can be adjusted from the GUI

    
    
baby_rowdy_control_v1

The new baby Rowdy robot has now Top body!

V1.0
July 7, 2024
    - Incorporate 3 motors of top body 
V1.1
July 9, 2024
    - Incorporate set points for each joint from User Interface via MQTT       
    
    
V2
July 25, 2024
    - Incorporate wheels control via MQTT on the UI (Visual Basic)
    - but this program DO NOT contol the Wheels!

    - A new program is generated (rollie_wheels_control v1.0 ) to control only the wheels. This is beause the wheels control slowers the legs control loop


v4
August 7, 2024
    - Improved MQTT SETUP configuration, to be equal to the other programs connecting to mosquitto
    - include anckles in Forward_step:  See program baby_rowdy_half_step_v2 in main computer for plotting the sinusoidal curves
    - The type of step is defined by the following parameters: 
        half_size_step = 2   #2 = complete step of two legs moving
        CM_frequency_factor = 1   
        
      - Add Jogging_step function both in raspberry and in VB Graphical User Interface
      - MAXCURRENT Protection 
            + The protection was NOT being executed!!
            + Now it is executed in the finction read_variables()  every 10MSec
            + Set to: MAX_CURRENT = 10.0


Git
August 15, 2024
    - First version that integrates the control program with the NVIDIA Isaac sim
    - There is  locl repository in the raspberry and a remote repository in: TBD!!         
To do..   
    - adition to sequential home roitine, one leg a a time
    - Equilibrium functions implemented

September 3, 2024
  - add neck control with RC radio
September 6, 2024
  - add read and write parameters from a file "/home/sergio/Projects/UTSA/baby_Rowdy/baby_Robot_body/release/settings.txt"
 
 October  2024
  - Reparir baby Rowdy motor M11.  It just stop functioning, not even turm on the blue led 
     Use  a new CubeMars controller V2.1     
  - add  M13 in GUI for big rowdy head  control
   
November 4, 2024
  - Reparir baby Rowdy motor M4.  It just stop functioning, not even turm on the blue led 
     Use  a new CubeMars controller V2.1  
   - Added strt up bypass circuit to avoid large strt inrush currents, that may be damagin the motors. 
   - To turn on the robot, first turn on the green led sw  and the the main red led sw. After 5 secs, turn off green led sw.     

   March 10,2025
    -  The HMI hs no tie to update all the variables that coe from the rasberry 
    - Change publish and command execution frequency to 200 mSeg in control_function_logic()
    - Also the HMI is updated to run the decodification every 200 mseg    
    - update the function def init_gains(), to manage different PD  gains per motor

    March 15,2025
    -  Remove the "isReal" variable and all the logic associated with it.
    -  This program control the real robot and the simulation  in  Isaac lab can run concurrently by subscribing to the topics of the HMI
    -  The HMI is updated to run the decodification every 200 mseg
    -  The Direct and Inverse kinemtics are updated to the simulations made in python


    March 17,2025
    -  Updated the definition and ipdate  of the Home_POSITIONS_dict dictionary and the HOME_POSITIONS list
    - Updated the definition and ipdate  of the ZERO_POSITIONS_dict dictionary and the ZERO_POSITIONS list
    - Updated the definition and ipdate  of the MOTOR_SIGN_dict dictionary and the MOTOR_SIGN list
    - Updated the definition and ipdate  of the HOME_ANGLES_dict dictionary and the HOME_ANGLES list
    - Updated the definition and ipdate  of the MOTOR_SIGN_dict dictionary and the MOTOR_SIGN list

    - correct bugs in the home function
    - correct bugs in the get_new_ang_position() function(XYZ)
    - increase the gain in Hip jaw motors




'''
import sys
import matplotlib.pyplot as plt
from time import sleep
from random import uniform
import numpy as np
import time
import math





from TMotorCANControl.mit_can import TMotorManager_mit_can
from witmotion import IMU


def millis():
    return round(time.time()*1000)

def callback_IMU(msg):
    global RPY_angles
    
    #print(msg)
    imu_string= imu.get_angle()
    #print (imu_string)
    #imu.get_angular_velocity()
    #imu.get_acceleration()
    

    roll, pitch, jaw = imu_string
    #print (f"Roll = {roll}, Pitch = {pitch}, Jaw = {jaw}")
    RPY_angles = roll, pitch, jaw 
    #print(RPY_angles)


##########################################################
# IMU setup
RPY_angles = [0.0, 0.0, 0.0]  # Initializing as a list with three float values

    
imu = IMU()
imu.subscribe(callback_IMU)




VERSION = "baby_rowdy_control"

file_path ="/home/sergio/Projects/UTSA/baby_Rowdy/baby_Robot_body/release/body_settings.txt"


print("\n baby Rowdy control program  Sergio Montufar UTSA")
print("\n Spring 2024 Version: ", VERSION)
    
state = False
debug = False


#T-Motors COnfiuration
##########################################################
Type_1 = 'AK60-6'


MAX_CURRENT = 10.0

##########################################################
# T-Motor  IDs and CAN addresses

ID_4 = 8    #left Hip jaw
ID_3 = 6    #left Hip pitch
ID_2 = 2    #left thigh
ID_1 = 3    #left knee
ID_11 = 12  #left anckle

ID_8 = 9   #right Hip jaw
ID_7 = 7   #right thigh
ID_6 = 4   #right upper_leg
ID_5 = 5   #right knee
ID_10 = 11 #right anckle

ID_9 = 10  # neck



joints_labels = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"] 
print("\n****************** Joint labels: ", joints_labels)



UP_DOWN_enable_joints = [1.0,1.0,0.0,0.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0]


# Zero (start up) positions definition
theta2_zero = 0.0
theta3_zero = 0.0
theta4_zero = 0.0
theta5_zero = 0.0
theta6_zero = 0.0
theta7_zero = 0.0
theta8_zero = 0.0   
theta9_zero = 0.0
theta10_zero = 0.0
theta11_zero = 0.0   
theta12_zero = 0.0

# Create a dictionary for the zero positions
ZERO_POSITIONS_dict = {
    "Theta2": theta2_zero,
    "Theta3": theta3_zero,
    "Theta4": theta4_zero,
    "Theta5": theta5_zero,
    "Theta6": theta6_zero,
    "Theta7": theta7_zero,
    "Theta8": theta8_zero,
    "Theta9": theta9_zero,
    "Theta10": theta10_zero,
    "Theta11": theta11_zero,
    "Theta12": theta12_zero
}

# Create a list for the zero positions
ZERO_POSITIONS = [ZERO_POSITIONS_dict[joint] for joint in joints_labels]  # List of zero positions in radians

# When the robot start, it goes to this position
desired_ang_position = ZERO_POSITIONS  #global variable containing the instantaneous desired position of all the robot joints
# Create a dictionary for the joints angles, which is updated to the desired_ang_position list every time a position is sent to the motors 
desired_ang_position_dict = dict(zip(joints_labels, ZERO_POSITIONS)) 

##########################################################
# Home positions definition
#The initial Home position is set in the angular space. 
#The robot must be in a vertical position when it is turned on

# Initial joint's home position angles in degrees
home_upper_leg_ang = 18
home_lower_leg_ang = 30
home_CM_displacement_ang  = 10
home_turn_ang = 0
home_step_neck_ang = 25
home_ankle_ang = 25




# Define home joint positions' angles in radians
HOME_ANGLES_dict = {
    "Theta3": np.radians(home_lower_leg_ang),  # left lower leg m3
    "Theta2": np.radians(home_upper_leg_ang),  # left upper leg m2
    "Theta6": np.radians(home_CM_displacement_ang),  # left lrg pitch m6
    "Theta8": np.radians(home_turn_ang),  # left leg pitch m8
    "Theta5": np.radians(home_lower_leg_ang),  # right lower leg m5
    "Theta4": np.radians(home_upper_leg_ang),  # right upper leg m4
    "Theta7": np.radians(home_CM_displacement_ang),  # right lrg pitch m7
    "Theta9": np.radians(home_turn_ang),  # right leg jaw m9
    "Theta10": np.radians(home_step_neck_ang),  # neck m10
    "Theta11": np.radians(home_ankle_ang),  # right ankle m11
    "Theta12": np.radians(home_ankle_ang),  # left ankle m12
}

# Define motor signs
MOTOR_SIGN_dict = {
    "Theta3": 1.0, #left Leg
    "Theta2": 1.0, #left Leg
    "Theta6": 1.0, #left Leg
    "Theta8": 1.0,#left Leg
    "Theta5": -1.0, #right Leg
    "Theta4": -1.0, #right Leg
    "Theta7": -1.0, #right Leg
    "Theta9": -1.0,#right Leg
    "Theta10": 1.0, #Neck
    "Theta11": -1.0, #right Leg
    "Theta12": 1.0#left Leg
}

# create List of motor signs
MOTOR_SIGN = [MOTOR_SIGN_dict[joint] for joint in joints_labels]  # List of motor signs

# Define home positions in radians
HOME_POSITIONS = [HOME_ANGLES_dict[joint] * MOTOR_SIGN_dict[joint] for joint in joints_labels]  # List of home positions in radians


print("\n****************** Default HOME_POSITIONS (in degrees): ",np.degrees(HOME_POSITIONS))

# Create a dictionary for the joints' home positions
HOME_POSITIONS_dict = {
    joint: HOME_ANGLES_dict[joint] * MOTOR_SIGN_dict[joint] for joint in joints_labels
}

# Example: Accessing data
# print(HOME_POSITIONS_dict["Theta3"])  # Get home position for Theta3 with motor sign applied

# Print the HOME_POSITIONS_dict in a formatted way
print("\n****************** Default HOME_POSITIONS (in radians): ")
for joint, value in HOME_POSITIONS_dict.items():
    print(f"{joint}: {value:.4f} radians")





    
# defaut step parameters
main_period =    2.0  # This is the period of the walking sequence 

step_upper_leg_ang = 10.0
step_lower_leg_ang = 30.0
step_CM_displacement_ang = 15.0
step_turn_ang = 0.0
step_neck_ang = 0.0
step_ankle_ang = 0.0

step_delay_sin_lower_leg = main_period/16 
step_delay_sin_upper_leg = main_period/8 
step_delay_sin_anckle = main_period/6 
step_delay_sin_neck = main_period/2 

#Step type
half_size_step = 2  # 2 = complete step of two legs moving
CM_frequency_factor = 1   

STEP_amplitudes = [step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_neck_ang,step_ankle_ang,step_ankle_ang]
STEP_amplitudes = np.radians(STEP_amplitudes)

jog_upper_leg_ang = 20.0
jog_lower_leg_ang = 20.0
jog_CM_displacement_ang = 10.0
jog_turn_ang = 0.0
jog_neck_ang = 0.0
jog_ankle_ang = 0.0

jog_delay_sin_lower_leg = main_period/8 
jog_delay_sin_upper_leg = main_period/8 
jog_delay_sin_anckle = main_period/6 
jog_delay_sin_neck = main_period/2 

Jogging_amplitudes = [jog_lower_leg_ang,jog_upper_leg_ang,jog_CM_displacement_ang,jog_turn_ang,jog_lower_leg_ang,jog_upper_leg_ang,jog_CM_displacement_ang,jog_turn_ang,jog_neck_ang,jog_ankle_ang,jog_ankle_ang]
Jogging_amplitudes = np.radians(Jogging_amplitudes)


SIDE_STEP_amplitudes = [0.5,0.5,-0.3,0.0,0.5,0.5,-0.3,0.0,0.0,0.0,0.0]

new_ang_position = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
old_ang_position  = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
variables_dict = {}
new_XYZ_position = [0.0,0.0,0.0]
old_XYZ_position = [0.0,0.0,0.0]
XYZ_dict ={}
RPY_dict={}

motor_currents = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
motor_torques = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
thetas_up_down = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

command = ""
trajectory_finished = True  #initial condition
trajectory = []
traj_idx = 0
num_cycles = 1
local_cycles = 0
direction = 1  # Direction of traversal (1 for increasing, -1 for decreasing)
idx_theta = 0



#array of tmmotorcancontrol objects
motors = [] 
#dictionary of tmmotorcancontrol objects
T_motors_dict = {}

positions_anterior = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

# settings is a global dictionary with all the  parameters  values
settings = {}

'''
settings = {
    'main_period': 2,
    'half_size_step': 2,
    
    'step_upper_leg_ang': 10,
    'step_lower_leg_ang': 30,
    'step_CM_displacement_ang': 15,
    'step_turn_ang': 0,
    'step_neck_ang': 0,
    'step_ankle_ang': 0,
    'step_delay_sin_lower_leg': main_period/16 ,
    'step_delay_sin_upper_leg': main_period/8 ,
    'step_delay_sin_ankle': main_period/6 ,
    'step_delay_sin_neck': main_period/2 ,
    
    'jog_delay_sin_lower_leg': main_period/8 ,
    'jog_delay_sin_upper_leg': main_period/8 ,
    'jog_delay_sin_anckle': main_period/6 ,
    'jog_delay_sin_neck': main_period/2 ,
    
    'jog_upper_leg_ang': 35,
    'jog_lower_leg_ang':35,
    'jog_CM_displacement_ang': 20,
    'jog_turn_ang': 0,
    'jog_neck_ang': 0,
    'jog_ankle_ang': 0,
    
    'home_upper_leg_ang': 18,
    'home_lower_leg_ang': 30,
    'home_CM_displacement_ang': 10,
    'home_turn_ang': 0,
    'home_ankle_ang': 25,
    'home_neck_ang': 25,
    

}
'''
##########################################################
#Read & write from settings.txt
def read_settings(file_path):
    """Reads the settings from settings.txt file and returns a dictionary of parameters and their values."""
    settings = {}
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if '=' in line and not line.startswith('#'):
                key, value = line.split('=', 1)
                settings[key.strip()] = value.strip()
    return settings

def write_settings_bak(file_path, settings):
    """Writes the provided settings dictionary to settings.txt file."""
    with open(file_path, 'w') as file:
        for key, value in settings.items():
            file.write(f"{key} = {value}\n")
            
            
def write_settings(file_path, settings):
    """Updates the values of the variables in the file without creating a new file."""
    # Read the current content of the file
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Create a dictionary to hold updated lines
    updated_lines = []

    # Update lines with the values from the settings dictionary
    for line in lines:
        key, sep, value = line.partition('=')
        key = key.strip()
        if key in settings:
            # Replace the value with the new one from the settings dictionary
            updated_line = f"{key} = {settings[key]}\n"
        else:
            # Keep the original line if the key isn't in the settings dictionary
            updated_line = line
        updated_lines.append(updated_line)

    # Write the updated lines back to the file
    with open(file_path, 'w') as file:
        file.writelines(updated_lines)
     


##########################################################
# MQTT setup
import paho.mqtt.client as mqtt
import ssl
import json


############################################################################
# MQTT Parameters & Functions

serverAddress = "localhost"
#serverAddress = "127.0.0.1"
# serverAddress = "rollie-body-pi"
#serverAddress = "baby-body-pi"
#serverAddress = "192.168.8.7"

# serverAddress, is the pi's host name. But, since our Mosquitto broker and
# this program (which acts as the subscriber) are on the same Raspberry Pi
# we can simply use "localhost" as the server name.

############################################################################
# MQTT Functions
# serverAddress, below is your pi's host name. But, since our Mosquitto broker and
# this program (which acts as the subscriber) are on the same Raspberry Pi
# we can simply use "localhost" as the server name.



def on_connect(client, userdata, flags, rc):
    global didPrintSubscribeMessage
    if not didPrintSubscribeMessage:
        didPrintSubscribeMessage = True
        print("subscribing")
        
        mqttClient.subscribe([ ("body/control/positionXYZ", 1), ("body/control/TrajectoryXYZ", 1), ("body/control/Neck_angle", 1), ("body/control/Set_angles", 1),("body/control/command", 1),("body/control/monitor", 1), ("body/control/exit", 1), ("body/control/walk_parameters" , 1), ("body/control/home_parameters" , 1)])

        print("subscribed")
        print("Connected to Mosquitto result code "+str(rc))
 # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    #AWS
    # client.subscribe([("$aws/things/raspberryPi4/shadow/update", 1), ("$aws/things/raspberryPi4/shadow/get", 1)])
    
def on_message(client, userdata, message):
    
 
    global new_XYZ_position
    global desired_ang_position
    global debug
    global salir
    global command
    global X,Y,Z
    global num_cycles

    global STEP_amplitudes
    global jogging_amplitudes
    global settings
    
    global home_upper_leg_ang
    global home_lower_leg_ang 
    global home_CM_displacement_ang 
    global home_turn_ang 
    global home_ankle_ang
    global home_neck_ang
    
    global step_upper_leg_ang 
    global step_lower_leg_ang 
    global step_CM_displacement_ang 
    global step_delay_sin_lower_leg  
    global step_delay_sin_upper_leg   
    global half_size_step 
    global step_turn_ang 
            
    global step_delay_sin_neck   
    global delay_sin_ankle             
    global step_neck_ang 
    global step_ankle_ang 
    global main_period 

    
    #print("Message received: " + message.topic + " : " + str(message.payload))
    #print("topic :",message.topic)
    #print(type(message.topic))      
    
    #print("payload :",message.payload)
    #print(type(message.payload)) 
       
    m_decode=str(message.payload.decode("utf-8","ignore"))
    #print("data Received type",type(m_decode))
    print("data Received",m_decode)
     
    #if message.topic == '$aws/things/raspberryPi4/shadow/update':
    if message.topic == 'body/control/positionXYZ' or message.topic == 'body/control/TrajectoryXYZ'  :    
        
        if message.topic == 'body/control/positionXYZ':
            print("body/control/positionXYZ recibido")
            command = "positionXYZ"
        if message.topic == 'body/control/TrajectoryXYZ':
            print("body/control/TrajectoryXYZ recibido")
            command = "TrajectoryXYZ"
        #print("Converting from Json to dictionary Object")
        m_in=json.loads(m_decode) #decode json data
        #print(type(m_in))
        #print(m_in)
        #print("Print each key-value pair from JSON response")
        #for key, value in m_in.items():
        #    print(key, ":", value)
        #    print('\n')

        if 'desired' in m_decode:    
            X = int((m_in["state"]["desired"]["X"]))
            
            Y = int((m_in["state"]["desired"]["Y"]))
            
            Z = int((m_in["state"]["desired"]["Z"]))
            
            
            new_XYZ_position = [X,Y,Z]

            
            if debug >= 1:
                print(" //// Desired position //// ")
                print("\n new_XYZ_position = ", new_XYZ_position)
                
    elif message.topic == 'body/control/Set_angles'  :    
        

        print("body/control/Set_angles recibido")
        

        #print("Converting from Json to dictionary Object")
        m_in=json.loads(m_decode) #decode json data
        #print(type(m_in))
        #print(m_in)
        #print("Print each key-value pair from JSON response")
        #for key, value in m_in.items():
        #    print(key, ":", value)
        #    print('\n')

        if 'desired' in m_decode:    
            Theta2 = float((m_in["state"]["desired"]["Theta2"]))
            Theta3 = float((m_in["state"]["desired"]["Theta3"]))
            Theta4 = float((m_in["state"]["desired"]["Theta4"]))
            Theta5 = float((m_in["state"]["desired"]["Theta5"]))
            Theta6 = float((m_in["state"]["desired"]["Theta6"]))
            Theta7 = float((m_in["state"]["desired"]["Theta7"]))
            Theta8 = float((m_in["state"]["desired"]["Theta8"]))
            Theta9 = float((m_in["state"]["desired"]["Theta9"]))
            Theta10 = float((m_in["state"]["desired"]["Theta10"]))
            Theta11 = float((m_in["state"]["desired"]["Theta11"]))
            Theta12 = float((m_in["state"]["desired"]["Theta12"]))
            
            
    
            
            
            Target_ang_position_deg = [ Theta3 ,  Theta2 ,  Theta6 ,  Theta8 ,  Theta5 ,  Theta4 ,  Theta7 ,  Theta9 ,  Theta10 ,  Theta11 ,  Theta12 ] 
            Target_ang_position =  [math.radians(angle) for angle in Target_ang_position_deg] 
            #desired_ang_position =  [math.radians(angle) for angle in desired_ang_position_deg] # if use global variable  desred_ang_position, This moves the Joints DIRECTLY!!, to quick for high gains!!!
            trajectory_ang_space(Target_ang_position) #here the movement is trough an angular trajectory in the angular space   
            if debug >= 1:
                print(" //// Desired angular positions //// ")
                print("\n desired_ang_position [deg] = ", Target_ang_position_deg)
                
            
    elif message.topic == 'body/control/Neck_angle'  :    
        

        print("body/control/Neck_angle recibido")
        

        #print("Converting from Json to dictionary Object")
        m_in=json.loads(m_decode) #decode json data
        #print(type(m_in))
        #print(m_in)
        #print("Print each key-value pair from JSON response")
        #for key, value in m_in.items():
        #    print(key, ":", value)
        #    print('\n')

        if 'desired' in m_decode:    

            Theta10_new_value = float((m_in["state"]["desired"]["Theta10"]))

            #Modify ONLY Theta10 value:
            #desired_ang_position_deg[8] = Theta10_new_value 

            # Update the value of Theta10 in the desired_ang_position_dict dictionary
            desired_ang_position_dict["Theta10"] = Theta10_new_value  # Theta10_new_value is in degrees

            # Convert the dictionary back to a list
            joints_labels = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"]
            desired_ang_position_deg_updated = [desired_ang_position_dict[angle] for angle in joints_labels]

            #Convert to radians
           # desired_ang_position =  [math.radians(angle) for angle in desired_ang_position_deg_updated] # if use global variable  desred_ang_position, This moves the Joints DIRECTLY!!, to quick for high gains!!!
            Target_ang_position =  [math.radians(angle) for angle in desired_ang_position_deg_updated]
            trajectory_ang_space(Target_ang_position) #here the movement is trough an angular trajectory in the angular space  
            
            if debug >= 1:
                print(" //// Desired angular positions //// ")
                print("\n desired_ang_position [deg] = ", desired_ang_position_deg_updated)
                
            

                             
    elif message.topic =="body/control/walk_parameters"   :    
        
        print("body/control/walk_parameters recibido")
        command = "walk_parameters"
        #print("Converting from Json to dictionary Object")
        m_in=json.loads(m_decode) #decode json data
        #print(type(m_in))
        #print(m_in)
        #print("Print each key-value pair from JSON response")
        #for key, value in m_in.items():
        #    print(key, ":", value)
        #    print('\n')

        if 'parameters' in m_decode:    
            
            step_upper_leg_ang = float((m_in["step"]["parameters"]["Upper_Leg_Ang"]))
            step_lower_leg_ang = float((m_in["step"]["parameters"]["Lower_Leg_Ang"]))
            step_CM_displacement_ang = float((m_in["step"]["parameters"]["CM_disp_Ang"]))
 
            step_delay_sin_lower_leg = float((m_in["step"]["parameters"]["Delay_Lower_leg"]))/1000  #This time comes in mseconds  
            step_delay_sin_upper_leg = float((m_in["step"]["parameters"]["Delay_Upper_leg"]))/1000  #This time comes in mseconds  
            
            half_size_step = int((m_in["step"]["parameters"]["Half_size_step"]))
            
            step_turn_ang = float((m_in["step"]["parameters"]["Turn_Ang"]))
            
            step_delay_sin_neck = float((m_in["step"]["parameters"]["Delay_Neck"]))/1000  #This time comes in mseconds  
            step_delay_sin_ankle = float((m_in["step"]["parameters"]["Delay_Ankle"])) /1000  #This time comes in mseconds            
            step_neck_ang =  float((m_in["step"]["parameters"]["Neck_Ang"]))
            step_ankle_ang = float((m_in["step"]["parameters"]["Ankle_Ang"]))    
            
            main_period = float((m_in["step"]["parameters"]["Main_period"]))/1000  #This time comes in mseconds     
              

            print(" //// step parameters: //// ")
            print("\r Main_period  [mSecs] = ", main_period*1000)
            print("\r Half_size_step = ", half_size_step)
            
            print("\r Upper_Leg_Ang = ", step_upper_leg_ang)
            print("\r Lower_Leg_Ang = ", step_lower_leg_ang)
            print("\r CM_disp_Ang = ", step_CM_displacement_ang)
            print("\r Turn_Ang = ", step_turn_ang)
            print("\r step_neck_ang = ", step_neck_ang)
            print("\r step_ankle_ang = ", step_ankle_ang)
            
            print("\r Delay_Lower_leg [mSecs] = ", step_delay_sin_lower_leg)
            print("\r Delay_Upper_leg [mSecs] = ", step_delay_sin_upper_leg)
            print("\r Delay_Neck [mSecs] = ", step_delay_sin_neck)
            print("\r Delay_Ankle [mSecs] = ", step_delay_sin_ankle)

            
            # Update the settings dictionary with the values of the individual variables
            settings = {
                'main_period': main_period,
                'half_size_step': half_size_step,

                'step_upper_leg_ang': step_upper_leg_ang,
                'step_lower_leg_ang': step_lower_leg_ang,
                'step_CM_displacement_ang': step_CM_displacement_ang,
                'step_turn_ang': step_turn_ang,
                'step_neck_ang': step_neck_ang,
                'step_ankle_ang': step_ankle_ang,
                
                'step_delay_sin_lower_leg': step_delay_sin_lower_leg,
                'step_delay_sin_upper_leg': step_delay_sin_upper_leg,
                'step_delay_sin_ankle': step_delay_sin_ankle,
                'step_delay_sin_neck': step_delay_sin_neck,
                
                # Include additional parameters as needed
            }
            
            #write the settings to settings.txt file
            write_settings(file_path, settings) 
            
            print("\n Updated settings dictionary:")
            # Print each item of the settings dictionary in a different row
            for key, value in settings.items():
                print(f"{key}: {value}")


            # recall the joints order : ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"]
            STEP_amplitudes = [step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_neck_ang,step_ankle_ang,step_ankle_ang]
            STEP_amplitudes = np.radians(STEP_amplitudes)
            
                                
    elif message.topic =="body/control/jog_parameters"   :    
        
        print("body/control/jog_parameters recibido")
        command = "jog_parameters"
        #print("Converting from Json to dictionary Object")
        m_in=json.loads(m_decode) #decode json data
        #print(type(m_in))
        #print(m_in)
        #print("Print each key-value pair from JSON response")
        #for key, value in m_in.items():
        #    print(key, ":", value)
        #    print('\n')

        if 'parameters' in m_decode:    
            
            jog_upper_leg_ang = float((m_in["jog"]["parameters"]["Upper_Leg_Ang"]))
            jog_lower_leg_ang = float((m_in["jog"]["parameters"]["Lower_Leg_Ang"]))
            jog_CM_displacement_ang = float((m_in["jog"]["parameters"]["CM_disp_Ang"]))
 
            jog_delay_sin_lower_leg = float((m_in["jog"]["parameters"]["Delay_Lower_leg"]))/1000  #This time comes in mseconds  
            jog_delay_sin_upper_leg = float((m_in["jog"]["parameters"]["Delay_Upper_leg"]))/1000  #This time comes in mseconds  
            
            half_size_jog = int((m_in["jog"]["parameters"]["Half_size_jog"]))
            
            jog_turn_ang = float((m_in["jog"]["parameters"]["Turn_Ang"]))
            
            jog_delay_sin_neck = float((m_in["jog"]["parameters"]["Delay_Neck"]))/1000  #This time comes in mseconds  
            jog_delay_sin_ankle = float((m_in["jog"]["parameters"]["Delay_Ankle"])) /1000  #This time comes in mseconds            
            jog_neck_ang =  float((m_in["jog"]["parameters"]["Neck_Ang"]))
            jog_ankle_ang = float((m_in["jog"]["parameters"]["Ankle_Ang"]))    
            
            main_period = float((m_in["jog"]["parameters"]["Main_period"]))/1000  #This time comes in mseconds     
              

            print(" //// jog parameters: //// ")
            print("\r Main_period  [mSecs] = ", main_period*1000)
            print("\r Half_size_jog = ", half_size_jog)
            
            print("\r Upper_Leg_Ang = ", jog_upper_leg_ang)
            print("\r Lower_Leg_Ang = ", jog_lower_leg_ang)
            print("\r CM_disp_Ang = ", jog_CM_displacement_ang)
            print("\r Turn_Ang = ", jog_turn_ang)
            print("\r jog_neck_ang = ", jog_neck_ang)
            print("\r jog_ankle_ang = ", jog_ankle_ang)
            
            print("\r Delay_Lower_leg [mSecs] = ", jog_delay_sin_lower_leg)
            print("\r Delay_Upper_leg [mSecs] = ", jog_delay_sin_upper_leg)
            print("\r Delay_Neck [mSecs] = ", jog_delay_sin_neck)
            print("\r Delay_Ankle [mSecs] = ", delay_sin_ankle)

            
            # Update the settings dictionary with the values of the individual variables
            settings = {
                'main_period': main_period,
                'half_size_jog': half_size_jog,

                'jog_upper_leg_ang': jog_upper_leg_ang,
                'jog_lower_leg_ang': jog_lower_leg_ang,
                'jog_CM_displacement_ang': jog_CM_displacement_ang,
                'jog_turn_ang': jog_turn_ang,
                'jog_neck_ang': jog_neck_ang,
                'jog_ankle_ang': jog_ankle_ang,
                
                'jog_delay_sin_lower_leg': jog_delay_sin_lower_leg,
                'jog_delay_sin_upper_leg': jog_delay_sin_upper_leg,
                'jog_delay_sin_ankle': jog_delay_sin_ankle,
                'jog_delay_sin_neck': jog_delay_sin_neck,
                
                # Include additional parameters as needed
            }
            
            #write the settings to settings.txt file
            write_settings(file_path, settings) 
            
            print("\n Updated settings dictionary:")
            # Print each item of the settings dictionary in a different row
            for key, value in settings.items():
                print(f"{key}: {value}")


            # recall the joints order : ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"]
            jogging_amplitudes = [jog_lower_leg_ang,-jog_upper_leg_ang,jog_CM_displacement_ang,jog_turn_ang,jog_lower_leg_ang,-jog_upper_leg_ang,jog_CM_displacement_ang,jog_turn_ang,jog_neck_ang,jog_ankle_ang,jog_ankle_ang]
            jogging_amplitudes = np.radians(jogging_amplitudes)
            
             
           
                
    elif message.topic =="body/control/home_parameters"   :    
        
        print("body/control/walk_parameters recibido")
        command = "walk_parameters"
        #print("Converting from Json to dictionary Object")
        m_in=json.loads(m_decode) #decode json data
        #print(type(m_in))
        #print(m_in)
        #print("Print each key-value pair from JSON response")
        #for key, value in m_in.items():
        #    print(key, ":", value)
        #    print('\n')

        if 'parameters' in m_decode:    
            home_upper_leg_ang = float((m_in["home"]["parameters"]["home_Upper_Leg_Ang"]))
            home_lower_leg_ang = float((m_in["home"]["parameters"]["home_Lower_Leg_Ang"]))
            home_CM_displacement_ang = float((m_in["home"]["parameters"]["home_CM_disp_Ang"]))
            home_turn_ang = int((m_in["home"]["parameters"]["home_Turn_Ang"]))
            home_ankle_ang = int((m_in["home"]["parameters"]["home_Ankle_Ang"]))
            home_neck_ang = int((m_in["home"]["parameters"]["home_Neck_Ang"]))
  

            print(" //// home parameters: //// ")
            print("\r home_Upper_Leg_Ang = ", home_upper_leg_ang)
            print("\r home_Lower_Leg_Ang = ", home_lower_leg_ang)
            print("\r home_CM_disp_Ang = ", home_CM_displacement_ang)
            print("\r home_turn_ang = ", home_turn_ang)
            print("\r home_ankle_ang = ", home_ankle_ang)
            print("\r home_neck_ang = ", home_neck_ang)
            
            update_home_positions()
            
            # Update the settings dictionary with the values of the individual variables, replacing 'step' with 'home'
            settings = {
                'home_upper_leg_ang': home_upper_leg_ang,
                'home_lower_leg_ang': home_lower_leg_ang,
                'home_CM_displacement_ang': home_CM_displacement_ang,
                'home_turn_ang': home_turn_ang,
                'home_neck_ang': home_neck_ang,
                'home_ankle_ang': home_ankle_ang,
                
                # 'home_delay_sin_lower_leg': home_delay_sin_lower_leg,
                # 'home_delay_sin_upper_leg': home_delay_sin_upper_leg,
                # 'home_delay_sin_ankle': home_delay_sin_ankle,
                # 'home_delay_sin_neck': home_delay_sin_neck,
                
                # Include additional parameters as needed
            }

            # Write the settings to settings.txt file
            write_settings(file_path, settings)

            print("Updated settings dictionary:")
            # Print each item of the settings dictionary in a different row
            for key, value in settings.items():
                print(f"{key}: {value}")
                           
    elif  message.topic == 'body/control/command':       
        
        
        if 'home' in m_decode:   #direct command, its not json data! 
            print("body/control/command  home recibido") 
            command = "home" 
        elif 'reset' in m_decode:    
            print("body/control/command  reset recibido") 
            command = "reset"   
        elif  'Forward_step' in m_decode:       
            print("body/control/command Forward_step recibido") 
            m_in=json.loads(m_decode) #decode json data
            if 'Forward_step' in m_decode:    
                num_cycles = int((m_in["Forward_step"]["parameter"]["cycles"]))
                print("cycles = ",num_cycles)
            command = "Forward_step" 
            print("Initalizing the Forward Step")
            setup_sinusolidal_step(1) #cycles must be integer
        elif  'Jogging_step' in m_decode:       
            print("body/control/command Jogging_step recibido") 
            m_in=json.loads(m_decode) #decode json data
            if 'Jogging_step' in m_decode:    
                num_cycles = int((m_in["Jogging_step"]["parameter"]["cycles"]))
                print("cycles = ",num_cycles)
            command = "Jogging_step" 
            print("Initalizing The Jogging Step")
            setup_sinusolidal_Jogging_step(1) #cycles must be integer 
        elif  'side_step' in m_decode:       
            print("body/control/command side_step recibido") 
            m_in=json.loads(m_decode) #decode json data
            if 'side_step' in m_decode:    
                num_cycles = int((m_in["side_step"]["parameter"]["cycles"]))
                print("cycles = ",num_cycles)
            command = "side_step"                
        elif  'up_down' in m_decode:       
            print("body/control/command up_down recibido") 
            m_in=json.loads(m_decode) #decode json data
            if 'up_down' in m_decode:    
                num_cycles = int((m_in["up_down"]["parameter"]["cycles"]))
                print("cycles = ",num_cycles)
            command = "up_down" 
            setup_up_down()
    elif  message.topic == 'body/control/monitor':   
          
        if  'angles' in m_decode:       
            print("body/control/monitor_angles recibido") 
            command = "monitor_angles"     
        elif  'XYZ'in m_decode:       
            print("body/control/monitor_XYZ recibido") 
            command = "monitor_XYZ"   
        elif  'RPY'in m_decode:       
            print("body/control/monitor_RPY recibido") 
            command = "monitor_RPY"  
        elif  'torques'in m_decode:       
            print("body/control/monitor_torques recibido") 
            command = "monitor_torques"     
        elif  'currents'in m_decode:       
            print("body/control/monitor_currents recibido") 
            command = "monitor_currents"              
    elif  message.topic == 'body/control/exit':       
        print("body/control/exit recibido") 
         
        salir = True   # al salir se va a Home!
    
# Function to publish message
def publish_message(client, topic, message):
    client.publish(topic, message)
    #print("Message published:", message)




def setup_mqtt():
    print('----------------------------------------')
    global didPrintSubscribeMessage
    global mqttClient

    didPrintSubscribeMessage = False
    # Set up calling functions to mqttClient
    mqttClient = mqtt.Client()
    mqttClient.on_connect = on_connect  # attach function to callback
    mqttClient.on_message = on_message  # attach function to callback



    print("mqtt server address is:", serverAddress)
    mqttClient.connect(serverAddress)
    
    # Connect to the MQTT server  in the local LAN & loop forever.
    # CTRL-C will stop the program from running.   
    #mqttClient.loop_forever()# use this line if you don't want to write any further code. It blocks the code forever to check for data

    # Start the MQTT client in a non-blocking thread
    mqttClient.loop_start() #use this line if you want to write any more code here to execute along the mqtt client

    print('----------------------------------------')


def stop_mqtt():
    global mqttClient
    global didPrintSubscribeMessage

    if(mqttClient is not None):
        print("No Longer Listening")
        mqttClient.disconnect()
        mqttClient.loop_stop()
        didPrintSubscribeMessage = False




##################################################################
# T-Motors control routines
def update_home_positions():
    global HOME_POSITIONS

    global settings
    global main_period
    global half_size_step

    #Update the settings from the file body_settings.txt
    settings = read_settings(file_path)
    
    # Assign dictionary values to individual variables
    main_period = settings['main_period']
    half_size_step = settings['half_size_step']
    
    NEW_HOME_dict = {
        "Theta3": np.radians(float(settings['home_lower_leg_ang'])),  # left lower leg m3
        "Theta2": np.radians(float(settings['home_upper_leg_ang'])),  # left upper leg m2
        "Theta6": np.radians(float(settings['home_CM_displacement_ang'])),
        "Theta8": np.radians(float(settings['home_turn_ang'])),
        "Theta5": np.radians(float(settings['home_lower_leg_ang'])),  # right lower leg m5
        "Theta4": np.radians(float(settings['home_upper_leg_ang'])),  # right upper leg m4
        "Theta7": np.radians(float(settings['home_CM_displacement_ang'])),
        "Theta9": np.radians(float(settings['home_turn_ang'])),
        "Theta10": np.radians(float(settings['home_neck_ang'])),  # neck
        "Theta11": np.radians(float(settings['home_ankle_ang'])),  # right ankle
        "Theta12": np.radians(float(settings['home_ankle_ang']))  # left ankle
    }

    # Update home positions List in radians
    HOME_POSITIONS = [NEW_HOME_dict[joint] * MOTOR_SIGN_dict[joint] for joint in joints_labels]  # List of home positions in radians


    print("\n****************** Updated HOME_POSITIONS (in degrees): ",np.degrees(HOME_POSITIONS))
    # Update HOME_POSITIONS_dict  
    HOME_POSITIONS_dict = {
        joint: NEW_HOME_dict[joint] * MOTOR_SIGN_dict[joint] for joint in joints_labels
    }

    # Example: Accessing data
    # print(HOME_POSITIONS_dict["Theta3"])  # Get home position for Theta3 with motor sign applied

    # Print the HOME_POSITIONS_dict in a formatted way
    print("\n****************** Updated HOME_POSITIONS (in radians): ")
    for joint, value in HOME_POSITIONS_dict.items():
        print(f"{joint}: {value:.4f} radians")

            
                   
def display_ang_positions():
    global old_ang_position
    global debug
    
    
    for idx, motor in enumerate(motors):
        #    motor.update
        motors[idx].update()
        
    total_current = 0.0    
    #joints_labels = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"]
    
    # Initialize an empty dictionary
    Thetas_dict = {}
    
    for idx, (key, motor) in enumerate(T_motors_dict.items()):
 
        #motors[idx].update()   
        #motor.update()
        #time.sleep(0.1)
        #theta = motors[idx].position   # Reporta un estado anterior
        theta = motor.position
        current = motor._motor_state.current
        total_current += current
        old_ang_position[idx] = theta+ ZERO_POSITIONS[idx]
            #Create a dictionary for publish values
        #Thetas_dict[joints_labels[idx]] = round(np.degrees(theta),2)
        Thetas_dict[key] = round(np.degrees(theta),2)
        
        if debug >= 3: 
            print("\r  theta", idx, theta,"rads ", np.degrees(theta), "degrees")
            print("\r  Current", idx, current,"Amps ")
            #print("\r  Temperature", idx, motor._motor_state.temperature," C ")  # entrega basura
           
        
    
    if debug >= 1:
        print("\r  ============================================================")
        print("\n  ->->->->->-> total_current ->->->->-> ", total_current)  
        print("\r  ============================================================")   
        print("\r  old_ang_position (rads)", old_ang_position)
            # Convert each float to a string with 2 decimal places
        formatted_numbers = map(lambda x: f"{x:.2f}", np.degrees(old_ang_position))
            # Join the formatted strings into a single string and print
        print(f"  old_ang_position (degrees): {', '.join(formatted_numbers)}")
        print("\r  ============================================================")
 
        #Publish the angles
    topic = "body/monitor/angles"
    print(Thetas_dict)
        # Serialize dictionary to JSON string with newline after each key-value pair
    json_string = json.dumps(Thetas_dict, indent=4)  # Pretty print with indentation
    json_string_with_newline = '\n'.join(json_string.split('\n'))

        # Publish JSON string to MQTT broker
    publish_message(mqttClient, topic, json_string_with_newline)
 
 
def display_XYZ_positions():
    global old_ang_position
    global old_XYZ_position
    global debug
    global XYZ_dict
    
    display_ang_positions()
    old_XYZ_position = forward_kinematics(old_ang_position)
    
    if debug >= 3:
        print("\r  ============================================================")
        print("\r  old_XYZ_position", old_XYZ_position)
        print("\r  ============================================================")  
        
    XYZ_dict["X"]=old_XYZ_position[0]
    XYZ_dict["Y"]=old_XYZ_position[1]
    XYZ_dict["Z"]=old_XYZ_position[2]     


            #Publish the XYZ positions
    topic = "body/monitor/XYZ"
    print(XYZ_dict)
        # Serialize dictionary to JSON string with newline after each key-value pair
    json_string = json.dumps(XYZ_dict, indent=4)  # Pretty print with indentation
    json_string_with_newline = '\n'.join(json_string.split('\n'))

        # Publish JSON string to MQTT broker
    publish_message(mqttClient, topic, json_string_with_newline)
        
def publish_variables():
    global debug
    global variables_dict
    
                #Publish the all variables
    topic = "body/monitor/all"
    if debug >= 2:
        print(variables_dict)
    
        # Serialize dictionary to JSON string with newline after each key-value pair
    json_string = json.dumps(variables_dict, indent=4)  # Pretty print with indentation
    json_string_with_newline = '\n'.join(json_string.split('\n'))

        # Publish JSON string to MQTT broker
    publish_message(mqttClient, topic, json_string_with_newline)
     
         
def read_variables():
    global motors
    global old_ang_position
    global old_XYZ_position
    global debug
    global motor_currents
    global variables_dict
    global RPY_angles 
    
    for idx, motor in enumerate(motors):
        #    motor.update
        motors[idx].update()
        
    total_current = 0.0    
    # Initialize an empty dictionary
    joints_labels = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"]
    currents_labels = ["I m3", "I m2", "I m6", "I m8", "I m5", "I m4", "I m7", "I m9", "I m10", "I m11", "I m12", "I tot"]
    torques_labels= ["Torque3", "Torque2", "Torque6", "Torque8", "Torque5", "Torque4", "Torque7", "Torque9", "Torque10", "Torque11", "Torque12"]
    variables_dict = {}
    
    for idx, motor in enumerate(motors):
 

        theta = motor.position
        current = abs(motor._motor_state.current)
        
        #Current protection  VERY IMPORTANT!
        if current > MAX_CURRENT:
            print(" \r *********  WARNING !! ***********************************************************")
            print("\r ******  MAX CURRENT EXEEDED !! *****  Motor:",currents_labels[idx], "Value: ",current)
            print(" \r *********************************************************************************")
            print("\n")
            sys.exit()
                
                
        torque = motor.torque
        total_current += current
        old_ang_position[idx] = theta+ ZERO_POSITIONS[idx]
            #Create a dictionary for publish values
        variables_dict[joints_labels[idx]] = round(np.degrees(theta),2)
        variables_dict[currents_labels[idx]] = round(current,2)
        variables_dict[torques_labels[idx]] = round(torque,2)

    old_XYZ_position = forward_kinematics(old_ang_position)
    variables_dict["X"]=old_XYZ_position[0]
    variables_dict["Y"]=old_XYZ_position[1]
    variables_dict["Z"]=old_XYZ_position[2] 
        
    variables_dict[ "I tot"] = round(total_current,2)   
    
    variables_dict["Roll"]=RPY_angles[0]
    variables_dict["Pitch"]=RPY_angles[1]
    variables_dict["Yaw"]=RPY_angles[2]     

    
    
    #publish_variables()


    if debug >= 3:
        print("\r  ============================================================")
        print("\n  ->->->->->-> total_current ->->->->-> ", total_current)  
        print("\r  ============================================================")   
        print("\r  old_ang_position (rads)", old_ang_position)
        print("\r  old_XYZ_position", old_XYZ_position)
        print("\r  ============================================================")      
  
def display_RPY_angles():

    global debug
    global RPY_dict
    global RPY_angles 
    

        
    RPY_dict["Roll"]=RPY_angles[0]
    RPY_dict["Pitch"]=RPY_angles[1]
    RPY_dict["Yaw"]=RPY_angles[2]     

    if debug >= 2:
        #print("\r  ============================================================")
        #print("\r  RPY_angles", RPY_angles)
        print(RPY_dict) 
        #print("\r  ============================================================")  
           
        
            #Publish the torques
    topic = "body/monitor/RPY"

        # Serialize dictionary to JSON string with newline after each key-value pair
    json_string = json.dumps(RPY_dict, indent=4)  # Pretty print with indentation
    json_string_with_newline = '\n'.join(json_string.split('\n'))

        # Publish JSON string to MQTT broker
    publish_message(mqttClient, topic, json_string_with_newline)
        
       
def display_ang_currents():
    global motor_currents
    global debug
    
    
    for idx, motor in enumerate(motors):
        #    motor.update
        motors[idx].update()
        
    total_current = 0.0   
        # Initialize an empty dictionary
    currents_labels = ["I m3", "I m2", "I m6", "I m8", "I m5", "I m4", "I m7", "I m9", "I m10", "I m11", "I m12", "I tot"]
    currents_dict = {} 
    for idx, motor in enumerate(motors):

        current = abs(motor._motor_state.current)
        motor_currents[idx] = current
        if current > MAX_CURRENT:
                print(" \r *********  WARNING !! ***********************************************************")
                print("\r ******  MAX CURRENT EXEEDED !! *****  Motor:",currents_labels[idx], "Value: ",current)
                print(" \r *********************************************************************************")
                print("\n")
                sys.exit()
        total_current += current
            #Create a dictionary for publish values
        currents_dict[currents_labels[idx]] = round(current,2)
        
        
        if debug >= 3: 
            print("\r  Current", idx, current,"Amps ")
            #print("\r  Temperature", idx, motor._motor_state.temperature," C ")  # entrega basura
            
    currents_dict[ "I tot"] = round(total_current,2)
    
    if debug >= 1:
        print("\r  ============================================================")
        print("\n  ->->->->->-> total_current ->->->->-> ", total_current)  
        print("\r  ============================================================")   

            # Convert each float to a string with 2 decimal places
        formatted_numbers = map(lambda x: f"{x:.2f}", motor_currents)
            # Join the formatted strings into a single string and print
          
        print(f"  motor_currents (Amps): {', '.join(formatted_numbers)}")
        print("\r  ============================================================")
        
            #Publish the currents
        topic = "body/monitor/currents"
        print(currents_dict)
            # Serialize dictionary to JSON string with newline after each key-value pair
        json_string = json.dumps(currents_dict, indent=4)  # Pretty print with indentation
        json_string_with_newline = '\n'.join(json_string.split('\n'))

            # Publish JSON string to MQTT broker
        publish_message(mqttClient, topic, json_string_with_newline)

       
def protect_max_currents():
    global motor_currents
    global debug
    
    
    for idx, motor in enumerate(motors):
        #    motor.update
        motors[idx].update()
        
    total_current = 0.0   

    for idx, motor in enumerate(motors):

        current = abs(motor._motor_state.current)
        motor_currents[idx] = current
        if current > MAX_CURRENT:
                print(" \r *********  WARNING !! ***********************************************************")
                print("\r ******  MAX CURRENT EXEEDED !! *****  Motor:",currents_labels[idx], "Value: ",current)
                print(" \r *********************************************************************************")
                print("\n")
                sys.exit()
        total_current += current

        
        
        if debug >= 3: 
            print("\r  Current", idx, current,"Amps ")
            #print("\r  Temperature", idx, motor._motor_state.temperature," C ")  # entrega basura
    if debug >= 1:
        print("\r  ============================================================")
        print("\n  ->->->->->-> total_current ->->->->-> ", total_current)  
        print("\r  ============================================================")    

    




def display_ang_torques():
    global motor_torques
    global debug
    
    
    for idx, motor in enumerate(motors):
        #    motor.update
        motors[idx].update()
        
  
        # Initialize an empty dictionary
    torques_labels= ["Torque3", "Torque2", "Torque6", "Torque8", "Torque5", "Torque4", "Torque7", "Torque9", "Torque10", "Torque11", "Torque12"]
    torques_dict = {} 
    for idx, motor in enumerate(motors):

        torque = motor.torque
        motor_torques[idx] = torque
        
            #Create a dictionary for publish values
        torques_dict[torques_labels[idx]] = round(torque,2)
        
        
        if debug >= 3: 
            print("\r  torque", idx, torque,"Amps ")
            #print("\r  Temperature", idx, motor._motor_state.temperature," C ")  # entrega basura
            
        
    
    if debug >= 1:


            # Convert each float to a string with 2 decimal places
        formatted_numbers = map(lambda x: f"{x:.2f}", motor_torques)
            # Join the formatted strings into a single string and print
          
        print(f"  motor_torques (New m): {', '.join(formatted_numbers)}")
        print("\r  ============================================================")
        
            #Publish the torques
        topic = "body/monitor/torques"
        print(torques_dict)
            # Serialize dictionary to JSON string with newline after each key-value pair
        json_string = json.dumps(torques_dict, indent=4)  # Pretty print with indentation
        json_string_with_newline = '\n'.join(json_string.split('\n'))

            # Publish JSON string to MQTT broker
        publish_message(mqttClient, topic, json_string_with_newline)
      
    
def init_motors():
    display_ang_positions()
    for idx, motor in enumerate(motors):
        print("\r  Setting zero motor ", idx)    
        motor.set_zero_position()
        time.sleep(1.5) # wait for the motors to zero (~1 second)
    display_ang_positions()  
   
################################################################################################################
##
# Forward and Inverse Kinematics
##

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

# Robot leg's dimensions
# kinematic model Link lengths

L1 = 5.411252782  # Thigh length
L2 = 5.411252782  # Shin length
L3 = 6.5  # Foot length rollie rowdy
L3 = 4.623258381  # Foot length baby rowdy




# Joint limits in Radians
thigh_min = np.radians(-120)  # Hip (thigh) angle range: -120° to -60°
thigh_max = np.radians(-60)
  
knee_min = np.radians(0)  # Knee angle range: 0° to 60°
knee_max = np.radians(60)


foot_absolute_angle = np.radians(-90)  # Foot must be perpendicular to the floor (-90°) 

#Auxiliary functions

def generate_reachability_space():
    """Generates all reachable (x, z) positions for the foot tip by sweeping thigh and knee angles."""
    angles_thigh = np.linspace(thigh_min, thigh_max, 50)  # 50 steps for thigh
    angles_knee = np.linspace(knee_min, knee_max, 50)  # 50 steps for knee

    reachable_points = []
    for theta1 in angles_thigh:
        for theta2 in angles_knee:
            pos = forward_kinematics_2D([theta1, theta2])
            reachable_points.append(pos)

    return np.array(reachable_points)

def is_position_reachable(target, reachable_space):
    """Checks if the given (x, z) target position is within the reachable space."""
    distances = np.linalg.norm(reachable_space - target, axis=1)
    min_distance = np.min(distances)
    return min_distance < 0.1  # Threshold for reachability check

def snap_to_nearest_valid_position(target, reachable_space):
    """Finds the closest valid point in the reachable space for the given target."""
    distances = np.linalg.norm(reachable_space - target, axis=1)
    nearest_index = np.argmin(distances)
    return reachable_space[nearest_index]

def plot_leg(theta, target, reachable_space, snapped_target=None):
    """Plots the 2D leg configuration and reachable space."""
    theta1, theta2, theta3 = theta

    # Compute joint positions
    x_hip, z_hip = 0, 0
    x_knee = x_hip + L1 * np.cos(theta1)
    z_knee = z_hip + L1 * np.sin(theta1)

    x_ankle = x_knee + L2 * np.cos(theta1 + theta2)
    z_ankle = z_knee + L2 * np.sin(theta1 + theta2)

    x_foot = x_ankle + L3 * np.cos(theta1 + theta2 + theta3)
    z_foot = z_ankle + L3 * np.sin(theta1 + theta2 + theta3)

    max_x = np.max(reachable_space[:, 0]) + 2  # Add tolerance to X limit

    plt.figure(figsize=(5, 5))
    plt.scatter(reachable_space[:, 0], reachable_space[:, 1], c='gray', s=5, alpha=0.3, label="Reachable Space")

    # Plot links
    plt.plot([x_hip, x_knee], [z_hip, z_knee], 'ro-', label="Thigh")   # Hip to Knee
    plt.plot([x_knee, x_ankle], [z_knee, z_ankle], 'bo-', label="Shin")  # Knee to Ankle
    plt.plot([x_ankle, x_foot], [z_ankle, z_foot], 'go-', label="Foot")  # Ankle to Foot
    
    # Plot joints
    plt.scatter([x_hip, x_knee, x_ankle, x_foot], [z_hip, z_knee, z_ankle, z_foot], c='k', label="Joints")

    # Plot desired and snapped target positions
    plt.scatter(target[0], target[1], c='r', marker='x', s=100, label="Original Target")
    if snapped_target is not None:
        plt.scatter(snapped_target[0], snapped_target[1], c='g', marker='o', s=100, label="Snapped Target (Valid)")

    plt.xlim(-max_x, max_x)
    plt.ylim(-20, 5)  # Fixed vertical range with a lower limit of -20
    plt.xlabel("X-axis")
    plt.ylabel("Z-axis")
    plt.title("2D Sagittal Leg with Foot Link & Snapped Target")
    plt.legend()
    plt.grid()
    plt.show()

    #-----------------------------------------------------------------------------------------------------------------

   #receives an angular position (rads) and delivers a XYZ position


#--------------------------------------------------------------
# Forward Kinematics 2D
# The forward kinematics problem is to compute the foot position (x, z) given the joint angles (thigh and knee).
# The function uses the law of cosines to compute the foot position based on the link lengths and joint angles.
# -------------------------------------------------------------


def forward_kinematics_2D(thetas):
    """Computes foot position (x, z) while keeping the foot perpendicular to the ground."""
    theta1, theta2 = thetas  # Hip and Knee angles
    theta3 = foot_absolute_angle - (theta1 + theta2)  # Enforce foot perpendicularity

    # Compute joint positions
    x_hip, z_hip = 0, 0  # Hip at origin

    x_knee = x_hip + L1 * np.cos(theta1)
    z_knee = z_hip + L1 * np.sin(theta1)

    x_ankle = x_knee + L2 * np.cos(theta1 + theta2)
    z_ankle = z_knee + L2 * np.sin(theta1 + theta2)

    x_foot = x_ankle + L3 * np.cos(theta1 + theta2 + theta3)
    z_foot = z_ankle + L3 * np.sin(theta1 + theta2 + theta3)

    return np.array([x_foot, z_foot])



#--------------------------------------------------------------
# Inverse Kinematics 2D
# The inverse kinematics problem is to find the joint angles that achieve a desired foot position (x, z)  
# The solution is not unique, and the leg configuration may vary depending on the initial guess and constraints.
# The function uses the scipy.optimize.least_squares method to solve the inverse kinematics problem.
# -------------------------------------------------------------    
#

def inverse_kinematics_2D(target, reachable_space):
    """Solves for joint angles given a target foot position (x, z)."""
    if not is_position_reachable(target, reachable_space):
        snapped_target = snap_to_nearest_valid_position(target, reachable_space)
        print("\n************************************************************************************************")
        print(f"WARNING: Target {target} is out of range. Snapping to nearest valid position {snapped_target}.")
        print("************************************************************************************************\n")
        target = snapped_target  # Adjust target

    def error_func(theta):
        return forward_kinematics_2D(theta) - target

    # Initial guess (hip at -90°, knee at 30°)
    theta_init = np.array([-np.radians(90), np.radians(30)])

    # Restrict angles: Hip (-120° to -60°), Knee (0° to 60°)
    try:
        result = least_squares(error_func, theta_init, bounds=(
            [thigh_min, knee_min],  # Lower bounds
            [thigh_max, knee_max]   # Upper bounds
        ))
    except Exception as e:
        raise ValueError(f"Least squares optimization failed: {e}")

    # Compute ankle angle and return
    ankle_angle = foot_absolute_angle - sum(result.x)
    
  
    print(f"IK Solution: Hip = {np.degrees(result.x[0])}°, Knee = {np.degrees(result.x[1])}°, Ankle = {np.degrees(ankle_angle)}°")
    
    return np.append(result.x, ankle_angle), target  # Append calculated ankle angle and return adjusted target


#_______________________________________________________________________________________________________________________


def forward_kinematics(current_position) :
# joints_labels = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"] 

    theta1= current_position[1]# theta 1 = Hip angle and theta 2 = Knee angle
    theta2 = -current_position[0] #the negative in link 2 (musle) is required for the correct conversion to vertical coordinate
 
     #adjust theta1 to be  -90 degrees when the joint angle in the robot is 0 degrees
    theta1 = theta1 - np.pi/2
    thetas = [theta1, theta2]
    # Compute foot position
    x_foot, z_foot = forward_kinematics_2D(thetas)
    y_foot = 0.0  # this is a 2d sagittal plane model


    return x_foot, y_foot, z_foot


def inverse_kinematics(x, z):
    error = False
    theta1, theta2, theta3 = None, None, None  # Initialize variables to avoid reference before assignment
    adjusted_target = None  # Initialize adjusted_target to avoid reference before assignment

    try:
        if z == 0:
            raise ValueError("some value of ({}, {}) is zero.".format(x, z))
        
        # Generate reachable space
        reachable_space = generate_reachability_space()

        # Create target position
        target_position = np.array([x, -z])  # Desired foot position (x, z)

        try:
            # Solve inverse kinematics
            theta_solution, adjusted_target = inverse_kinematics_2D(target_position, reachable_space)

            print("theta_solution, adjusted_target ", theta_solution, adjusted_target)  # adjusted_target is the target position after snapping to the nearest valid position

            # Convert radians to degrees
            theta_solution_degrees = np.degrees(theta_solution)

            # Compute foot position using the solved joint angles
            computed_position = forward_kinematics_2D(theta_solution[:2])

            # Print results
            print("Inverse Kinematics Solution (Hip, Knee, Ankle) in Degrees:", theta_solution_degrees)
            print("Desired Foot Position (x, z):", target_position)
            print("Computed Foot Position (x, z):", computed_position)
            print("Position Error (x, z):", np.abs(target_position - computed_position))

            theta1 = theta_solution[0]  # Hip angle
            theta2 = theta_solution[1]  # Knee angle
            theta3 = theta_solution[2]  # Ankle angle
            theta1 = theta1 + np.pi / 2  # Adjust theta1 to be zero when it is at -90 degrees

        except ValueError as e:
            print(e)
            error = True

    except ValueError:
        print("\n Error: Invalid inputs, either zero or too large.")
        error = True

    if adjusted_target is not None:
        print("Adjusted Target Position (x, z):", adjusted_target, "remove the Inverse Kinematics error since an adjusted target was found")
        error = False
    else:
        error = True
    
    return theta1, theta2, theta3, error    
 
################################################################################################################


def init_gains():
    print("\n Init motor gains")
    
    # Define motor angle names
    # joints_labels = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"]

    # Dictionary to set gains for each motor
    gains_dict = {
        "Theta3": {"K": 20.0, "B": 0.5},#left lower leg m3 (knee)
        "Theta2": {"K": 15.0, "B": 0.5},#left upper leg m2 (thigh)
        "Theta6": {"K": 20.0, "B": 0.5},#left hip pitch (CM displacement) m6
        "Theta8": {"K": 15.0, "B": 0.5},#left hip jaw m8
        "Theta5": {"K": 20.0, "B": 0.5},#right lower leg m5 (knee)
        "Theta4": {"K": 15.0, "B": 0.5},#right upper leg m4 (thigh)
        "Theta7": {"K": 20.0, "B": 0.5},#right hip pitch (CM displacement) m7
        "Theta9": {"K": 15.0, "B": 0.5},#right hip jaw m9
        "Theta10": {"K": 15.0, "B": 0.5},#neck m10
        "Theta11": {"K": 20.0, "B": 0.5},  # right ankle m11
        "Theta12": {"K": 20.0, "B": 0.5}   # left ankle m12
    }

    # Assign gains to each motor
    for idx, motor in enumerate(motors):
        theta_key = joints_labels[idx]  # Map motor index to the corresponding angle name
        K_val = gains_dict[theta_key]["K"]
        B_val = gains_dict[theta_key]["B"]
        
        print(f"Setting gains for motor {idx} ({theta_key}) -> K: {K_val}, B: {B_val}") 
        
        motor.set_impedance_gains_real_unit(K=K_val, B=B_val)

# receives the desired angular position (in radians) and Sends them to the motors  
def set_ang_position(positions):
    
    global debug
    global positions_anterior
    global desired_ang_position_deg

    # Define two np arrays for element-wise compaison
    array1 = np.array(positions)
    array2 = np.array(positions_anterior)

    # Check if array1 is not equal to array2
    if not np.array_equal(array1, array2):

        positions_anterior = positions
        if debug >= 2:
            print("\n **** set_ang_position function ****")
            #print("\n   Desired positions (rads):  ", positions, "rads")    
            # Convert each float to a string with 2 decimal places
            formatted_numbers = map(lambda x: f"{x:.2f}", np.degrees(positions))
            # Join the formatted strings into a single string and print
            print(f" Desired positions (degrees): {', '.join(formatted_numbers)}")  

            # Update desired_ang_position_dict with the desired position by mapping each angle name to its corresponding value
            joints_labels = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"]
            desired_ang_position_dict = dict(zip(joints_labels, positions))


        #Send the Angle SETPOINTS to the motors:    
        angles = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]   


        for idx, motor in enumerate(motors):
            angles[idx] =  ZERO_POSITIONS[idx]+ positions[idx]
            motor.position =  angles[idx]
            motor.update


#Moves to a XYZ position by using the inverse kinematic model, no trajectory!
def set_XYZ_position(XYZ_position):
    

    new_ang_position, error = get_new_ang_position(XYZ_position)
    if error:
        print("\r Error in inverse kinematic ")
    else: 
        
        set_ang_position(new_ang_position) 
        display_XYZ_positions()            
                    
                    
                    
 #Moves to a XYZ position by using a trajectory of traj_num_steps points ans using the inverse kinematic model  
 #This function eters and leaves moving point by point every time it is called (every Period)  
 #The trajectory begins at the currrent XYZ position, named old_XYZ_position
def trajectory_XYZ(XYZ_position):
    print("\r**** Trajectory ****")
    global old_XYZ_position
    global command
    
    
    display_XYZ_positions()   #For updating old_XYZ_position
    
     # Define the number of steps in the trajectory
    traj_num_steps = 100
    traj_idx= 0
    Period = 10
    Millis_ant = 0
    # Define the start and end points of the trajectory use npumpy arrays for convenience
    start_point = np.array(old_XYZ_position)  # [x, y, z]
    end_point = np.array(XYZ_position)    # [x, y, z]
        # Generate intermediate points along the trajectory using linear interpolation
    trajectory = np.linspace(start_point, end_point, traj_num_steps)

    while traj_idx < len(trajectory):
        
        CurrentMillis = millis()
        if (CurrentMillis - Millis_ant) >= Period:  # Entra cada "Period" segundos" 
            Millis_ant = CurrentMillis
                             
            set_XYZ_position(trajectory[traj_idx])
            traj_idx+=1 
            

    print("Trajectory finished")
    traj_idx= 0
 
    command = "" #This makes sure the trajectory_XYZ function is not called indefinetly


                           
def get_new_ang_position(XYZ_position):
    ''' receives aa desired XYZ position and delivers a new angular position  '''
    # global HOME_POSITIONS
    # update_home_positions # why do I have to do this?

    print("\n  Desired XYZ position: ", XYZ_position)
    
    x = XYZ_position[0] 
    y = XYZ_position[1]
    z = XYZ_position[2]
   
      # get the  angles (in rads) with respect to horizontal on the sagital plane by using the inverse kinematic model
    angle_hip, angle_knee, angle_ankle, error = inverse_kinematics(x, z) #angle1 = hip, angle2 = knee, angle3 = ankle
    
    if not error:# if the inverse kinematic model was successful

    
        if debug >= 1:
            print(f"IK Solution: Hip = {np.degrees(angle_hip)}°, Knee = {np.degrees(angle_knee)}°, Ankle = {np.degrees(angle_ankle)}°")
            print("\n Angles of inverse model -respect to vertical-")
            print("Theta1 (hip) =", np.degrees(angle_hip), "degrees")
            print("Theta2 (knee) =", np.degrees(angle_knee), "degrees")
            print("Theta3 (ankle) =", np.degrees(angle_ankle), "degrees")
            

        
            #Recall that the angles of the robot are defined as follows:
            #joints_labels = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"] 
            #MOTOR_SIGN = { "Theta3": 1.0, "Theta2": 1.0, "Theta6": 1.0, "Theta8": 1.0,    "Theta5": -1.0, "Theta4": -1.0, "Theta7": -1.0, "Theta9": -1.0, 
            #    "Theta10": 1.0, "Theta11": -1.0, "Theta12": 1.0 }

        new_position_dict = HOME_POSITIONS_dict.copy()

        new_ang_position = HOME_POSITIONS.copy()  # to change body height, the robot MUST be in the HOME position!!

        new_position_dict["Theta2"] = -angle_hip * MOTOR_SIGN_dict["Theta2"]  # Theta2
        new_position_dict["Theta3"] = angle_knee * MOTOR_SIGN_dict["Theta3"]  # Theta3

        new_position_dict["Theta4"] = -angle_hip * MOTOR_SIGN_dict["Theta4"]  # Theta4
        new_position_dict["Theta5"] = angle_knee * MOTOR_SIGN_dict["Theta5"]  # Theta5

        new_position_dict["Theta12"] = -angle_ankle * MOTOR_SIGN_dict["Theta12"]  # Theta12
        new_position_dict["Theta11"] = -angle_ankle * MOTOR_SIGN_dict["Theta11"]  # Theta11

        # Update new_ang_position list based on new_position_dict
        for i, joint in enumerate(joints_labels):
            new_ang_position[i] = new_position_dict[joint]


    else:# if the inverse kinematic model was NOT successful
        new_ang_position = old_ang_position
        error = True
        print("Error in inverse kinematic")   
    
        
    return new_ang_position, error
                    


def trajectory_ang_XYZ(XYZ_position):
    ''' Moves to a XYZ position by using a trajectory of traj_num_steps points  using the inverse kinematic model  
    #This function enters and leaves moving point by point every time it is called (every Period) 
    #The trajectory begins at the curent angular position called old_ang_position'''

    print("\r**** Trajectory_ang_XYZ initiated ****")
  
    #Get the angular position corresponding to the desired XYZ position
    new_ang_position, error = get_new_ang_position(XYZ_position) #this calls the inverse kinematic model 
    if error:
        print("\r Error in inverse kinematic, abort command! ")
        return  # Quit the function early  
        
    trajectory_ang_space(new_ang_position)
         
   #This function eters and leaves moving point by point every time it is called (every Period)          
def execute_trajectory():
    global old_ang_position
    global desired_ang_position
    global traj_idx
    global trajectory  
    global  trajectory_finished
    global debug
     
    if traj_idx < len(trajectory):
        old_ang_position =   trajectory[traj_idx]         
        set_ang_position(old_ang_position)
        if debug >= 1:
                print("\r ===============================>>>>>>>>>>>>>>>>>>>  Executing trajectory  Traj_idx = ", traj_idx)
        traj_idx+=1 
    else:
        traj_idx= 0
        trajectory = []
        trajectory_finished = True
        desired_ang_position = old_ang_position
        print("\r =====================================================================================================")
        print("\rtrajectory_ang_space finished \n")    
        print("\r =====================================================================================================")
        
 #Moves to a joints position by using a trajectory of traj_num_steps points in angular space. It DOES NOT use the inverse kinematic model  !
  #This function enters and leaves moving point by point every time it is called (every Period) 
 #The trajectory begins at the cuurent angular position called old_ang_position
def trajectory_ang_space(local_new_ang_position):
    print("\r**** >>>>>>>>>>>>>>>> trajectory_ang_space initiated  >>>>>>>>>>>>>>>>>>>>****")
    global old_ang_position
    global command
    global debug
    global traj_idx
    global trajectory
    global  trajectory_finished
    
    

    #display_XYZ_positions()   #For updating old_ang_position  NO LONGER NECCESARY IN V11
     # Define the number of steps in the trajectory
    traj_num_steps = 100
    traj_idx= 0
    trajectory_finished = False
    # Define the start and end points of the trajectory use npumpy arrays for convenience
    start_point = np.array(old_ang_position)  # [8 angles]


    end_point = np.array(local_new_ang_position)   # [8 angles]


        # Generate intermediate points along the trajectory using linear interpolation
        
    if debug >= 1:

                # Create dictionaries for start and end points
        start_point_dict = dict(zip(joints_labels, start_point))
        end_point_dict = dict(zip(joints_labels, end_point))

        # Print start and end points side by side
        # print("\nStart Point (radians) and End Point (radians):")
        # for joint in joints_labels:
        #     print(f"{joint}: {start_point_dict[joint]:.4f} -> {end_point_dict[joint]:.4f}")

        # Print start and end points side by side in degrees
        print("\nStart Point (degrees) and End Point (degrees):")
        for joint in joints_labels:
            print(f"{joint}: {np.degrees(start_point_dict[joint]):.4f}° -> {np.degrees(end_point_dict[joint]):.4f}°")


        
        print("\n   start_point (rads):  ", start_point, "rads")    
        # Convert each float to a string with 2 decimal places
        formatted_numbers = map(lambda x: f"{x:.2f}", np.degrees(start_point))
        # Join the formatted strings into a single string and print
        print(f" start_point (degrees): {', '.join(formatted_numbers)}")  
        
        print("\n   end_point (rads):  ", end_point, "rads")    
        # Convert each float to a string with 2 decimal places
        formatted_numbers = map(lambda x: f"{x:.2f}", np.degrees(end_point))
        # Join the formatted strings into a single string and print
        print(f" end_point (degrees): {', '.join(formatted_numbers)}")   
        
     #Create a trajectory for EACH JOINT!           
    trajectory = np.linspace(start_point, end_point, traj_num_steps)
    command = "" 

def home():
    global HOME_POSITIONS
    update_home_positions() # why do I have to do this?

    print("\n****************** HOME_POSITIONS: ",np.degrees(HOME_POSITIONS))
    #set_ang_position(HOME_POSITIONS)
    
    trajectory_ang_space(HOME_POSITIONS)

def reset():
    global ZERO_POSITIONS
    #set_ang_position(HOME_POSITIONS)
    trajectory_ang_space(ZERO_POSITIONS)    
   
   
def setup_up_down():
    global thetas_up_down
    
    print("\r ***** up_down *****")    
          # Generate thetas values
    delta_angle = np.pi/6.0
    resolution = 100
    thetas_up_down = np.linspace(0.0,delta_angle , resolution)  # 100 points from 0 to delta_angle . Importante empezar desde ZERO, para evitar un brinco a la primera posicion!!
   
      
   
# This function enters and leaves every Period, Howeve, it CREATES its own trajector, that is: it does not call  trajectory_ang_space(positions)
#It sends position directly to de motors                             
def up_down():
    global debug
    global num_cycles
    global local_cycles
    global HOME_POSITIONS
    global UP_DOWN_enable_joints
    update_home_positions  # why do I have to do this?
    
    global thetas_up_down
    global idx_theta
    global direction
    global desired_ang_position
    global command

    
    
    if local_cycles < num_cycles:
        

             
            theta = thetas_up_down[idx_theta] 
            #print("\r idx_theta",idx_theta)   
            #print ("\r theta",theta)  
            
            new_ang_positions = []
            for idx, angle in enumerate (HOME_POSITIONS):
                sign = MOTOR_SIGN[idx]*UP_DOWN_enable_joints[idx]
                #add theta to every motor angle position in HOME_POSITIONS, mask the angles with the enabled joints and  the correct sign
                new_ang_positions.append(angle  + sign*theta) 
            #set_ang_position(new_ang_positions)
            desired_ang_position = new_ang_positions
            
            idx_theta += direction
            if idx_theta >= len(thetas_up_down):
                direction = -1  # Reverse direction
                idx_theta = len(thetas_up_down) - 2  # Start from the second-to-last element
            elif idx_theta < 0:
                direction = 1  # Reverse direction
                idx_theta = 1  # Start from the second element
                local_cycles+=1 
                print("\r local_cycles",local_cycles)      
    else:
        local_cycles = 0
        direction = 1  # Direction of traversal (1 for increasing, -1 for decreasing)
        idx_theta = 0
        command = ""   # VERY IMPORTANT, to finish the movement!


  #This function created the trajectory for jogging
  # - Creates sinusoidal trajectories for the joints
  # - The speed of the movement is controlled by    main_period  
def setup_sinusolidal_Jogging_step(cycles):
    global sin_trajectory

    global main_period
    global half_size_step
    global CM_frequency_factor 
    

    global HOME_POSITIONS
    update_home_positions  # why do I have to do this? #read settings from the settings.txt file
   
    print("\r ***** setup_sinusolidal_Jogging_step *****")
   
   # Parameters
    sampling_time = 0.010 

    #read settings from the settings.txt file
    #settings = read_settings(file_path)
    
    # Assign dictionary values to individual variables
    main_period = float(settings['main_period'])
    half_size_step = float(settings['half_size_step'])

    jog_upper_leg_ang = float(settings['jog_upper_leg_ang'])
    jog_lower_leg_ang = float(settings['jog_lower_leg_ang'])
    jog_CM_displacement_ang = float(settings['jog_CM_displacement_ang'])
    jog_turn_ang = float(settings['jog_turn_ang'])
    jog_neck_ang = float(settings['jog_neck_ang'])
    jog_ankle_ang = float(settings['jog_ankle_ang'])
    
    jog_delay_sin_lower_leg = float(settings['jog_delay_sin_lower_leg'])
    jog_delay_sin_upper_leg = float(settings['jog_delay_sin_upper_leg'])
    jog_delay_sin_ankle = float(settings['jog_delay_sin_ankle'])
    jog_delay_sin_neck = float(settings['jog_delay_sin_neck'])
    
    # Define parameters
    #main_period = 2.0   # global variable: this is the period of the first sinusoidal function
    # Frequency of the first sinusoidal function
    main_frequency = 1 /  main_period

    amplitude1 = 1.0  # amplitude of the first sinusoidal function
    amplitude2 = 1.0  # amplitude of the second sinusoidal function
    amplitude3 = 1.0  # amplitude of the third sinusoidal function
    amplitude4 = 1.0  # amplitude of the third sinusoidal function
    
    #NO DELAY on the lower limbs movement for jogging!
    #  lower leg sinusoid2 delay in seconds, this is a global variable
    # jog_delay_sin_lower_leg = main_period/16
    
    # Calculate the frequency of the second sinusoidal function
    if jog_delay_sin_lower_leg == main_period/2:
        frequency2 = main_frequency *2  # Set a default frequency if delay equals period
    else:
        frequency2 =( 1 / (main_period/2 - jog_delay_sin_lower_leg))/2



    #NO DELAY on the upper limbs movement for jogging!
        # Upper leg sinusoid3 delay in seconds, this is a global variable
    #jog_delay_sin_upper_leg = main_period/8

    # Calculate the frequency of the third sinusoidal function
    if jog_delay_sin_upper_leg == main_period/2:
        frequency3 = main_frequency *2  # Set a default frequency if delay equals period
    else:
        frequency3 =( 1 / (main_period/2 - jog_delay_sin_upper_leg))/2


    #NO DELAY on the ankle  movement for jogging!
        # Upper leg sinusoid4 delay in seconds, this is a global variable
    #jog_delay_sin_ankle = main_period/16
    
    # Calculate the frequency of the fourth sinusoidal function
    if jog_delay_sin_upper_leg == main_period/2:
        frequency4 = main_frequency *2  # Set a default frequency if delay equals period
    else:
        frequency4 =( 1 / (main_period/2 - jog_delay_sin_anckle))/2


    # Time values
    
    #duration = num_cycles* Period  # Duration of the trajectory (in seconds)
    duration = main_period/2  # Duration of the trajectory (in seconds)
    num_samples = int(duration/ sampling_time) # Number of samples
    #num_samples = 1000
    print("num_samples:",num_samples )
    phase = 0.0 # np.pi / 2   # Phase shift (in radians)


    # Time array
    t = np.linspace(0, duration, num_samples)

    # Create sinusoidal data for both functions without delay
    sinusoid11 = amplitude1 * np.sin(2 * np.pi * main_frequency * t*CM_frequency_factor)
    sinusoid21 = amplitude2 * np.sin(2 * np.pi * frequency2 * t)
    sinusoid31 = amplitude3 * np.sin(2 * np.pi * frequency3 * t)
    sinusoid41 = amplitude4 * np.sin(2 * np.pi * frequency4 * t)


    
    # Calculate delay in samples
    delay_samples_sinusoid2 = int(jog_delay_sin_lower_leg / (t[1] - t[0]))

    # Apply delay to the second sinusoid
    sinusoid21_delayed = np.roll(sinusoid21, delay_samples_sinusoid2)

    # Adjust the delayed sinusoid to start from zero
    sinusoid21_delayed[:delay_samples_sinusoid2] = 0




    
    # Calculate delay in samples
    delay_samples_sinusoid3 = int(jog_delay_sin_upper_leg / (t[1] - t[0]))

    # Apply delay to the second sinusoid
    sinusoid31_delayed = np.roll(sinusoid31, delay_samples_sinusoid3)

    # Adjust the delayed sinusoid to start from zero
    sinusoid31_delayed[:delay_samples_sinusoid3] = 0

    
    # Calculate delay in samples
    delay_samples_sinusoid4 = int(jog_delay_sin_anckle / (t[1] - t[0]))

    # Apply delay to the second sinusoid
    sinusoid41_delayed = np.roll(sinusoid41, delay_samples_sinusoid4)

    # Adjust the delayed sinusoid to start from zero
    sinusoid41_delayed[:delay_samples_sinusoid4] = 0



    # **** Concatenate both half_size_step *****

    if half_size_step == 2:
        sinusoid1 = np.concatenate((sinusoid11,-sinusoid11))
        sinusoid2 = np.concatenate((sinusoid21_delayed,-sinusoid21_delayed))
        sinusoid3 = np.concatenate((sinusoid31_delayed,-sinusoid31_delayed))
        sinusoid4 = np.concatenate((sinusoid41_delayed,-sinusoid41_delayed))
    # **** left half step *****

    if half_size_step == 1:
        sinusoid1 = sinusoid11
        sinusoid2 = sinusoid21_delayed
        sinusoid3 = sinusoid31_delayed
        sinusoid4 = sinusoid41_delayed
        
    total_duration = cycles * main_period       # Duration of the trajectory (in seconds)
    total_num_samples = cycles * 2* num_samples   # Number of samples


    #**** Repeat the steps n times ****
    # Repeat the array n times
    sinusoid1 = np.tile(sinusoid1, cycles)
    sinusoid2 = np.tile(sinusoid2, cycles)
    sinusoid3 = np.tile(sinusoid3, cycles)
    sinusoid4 = np.tile(sinusoid4, cycles)

    duration2 = cycles * half_size_step*duration
    num_samples2 = int(cycles * half_size_step*num_samples)
    t2 = np.linspace(0, duration2, num_samples2)

    zeros_trajectory = np.zeros(num_samples2)
    # Stack sinusoidal data into columns of a global array
    sin_trajectory = np.column_stack(( sinusoid2, sinusoid3, sinusoid1, zeros_trajectory, sinusoid2, sinusoid3, sinusoid1, zeros_trajectory, sinusoid1, sinusoid4, sinusoid4))
    print("SIN TRAJECTORY INITIALIZED", sin_trajectory)
  
    #Plotting the sinusoidal functions from the global array
    # plt.figure(figsize=(10, 6))
    # plt.plot(t2, sin_trajectory[:, 0], label=f'Sinusoid M3 - {frequency2} Hz')
    # plt.plot(t2, sin_trajectory[:, 1], label=f'Sinusoid M2 - {frequency3} Hz')
    # plt.plot(t2, sin_trajectory[:, 2], label=f'Sinusoid M6 - {main_frequency} Hz')
    # plt.plot(t2, sin_trajectory[:, 3], label=f'Sinusoid M9 - {0} Hz')
    # plt.plot(t2, sin_trajectory[:, 4], label=f'Sinusoid M5 - {frequency2} Hz')
    # plt.plot(t2, sin_trajectory[:, 5], label=f'Sinusoid M4 - {frequency3} Hz')
    # plt.plot(t2, sin_trajectory[:, 6], label=f'Sinusoid M7 - {main_frequency} Hz')
    # plt.plot(t2, sin_trajectory[:, 7], label=f'Sinusoid M8 - {0} Hz')
    # plt.plot(t2, sin_trajectory[:, 8], label=f'Sinusoid M10 - {main_frequency} Hz')
    # plt.plot(t2, sin_trajectory[:, 9], label=f'Sinusoid M12 - {frequency4} Hz')
    # plt.plot(t2, sin_trajectory[:, 10], label=f'Sinusoid M11 - {frequency4} Hz')


    # plt.title('Four Sinusoidal Functions with delay  Shift')
    # plt.xlabel('Time (seconds)')
    # plt.ylabel('amplitude')
    # plt.legend()
    # plt.grid(True)
    # plt.show()





    print("sin_trajectory:",sin_trajectory )    
    idx_theta = 0     
 

    
    
def setup_sinusolidal_step(cycles):
    global sin_trajectory

    global main_period
    global half_size_step

    global CM_frequency_factor 
    global settings
    global HOME_POSITIONS
    update_home_positions  # why do I have to do this? #read settings from the settings.txt file
   
    print("\r ***** setup_sinusolidal_step *****")
   
   # Parameters
    sampling_time = 0.010 
    
    #read settings from the settings.txt file
    #settings = read_settings(file_path)
    
    # Assign dictionary values to individual variables
    main_period = float(settings['main_period'])
    half_size_step = float(settings['half_size_step'])

    step_upper_leg_ang = float(settings['step_upper_leg_ang'])
    step_lower_leg_ang = float(settings['step_lower_leg_ang'])
    step_CM_displacement_ang = float(settings['step_CM_displacement_ang'])
    step_turn_ang = float(settings['step_turn_ang'])
    step_neck_ang = float(settings['step_neck_ang'])
    step_ankle_ang = float(settings['step_ankle_ang'])
    step_delay_sin_lower_leg = float(settings['step_delay_sin_lower_leg'])
    step_delay_sin_upper_leg = float(settings['step_delay_sin_upper_leg'])
    step_delay_sin_ankle = float(settings['step_delay_sin_ankle'])
    step_delay_sin_neck = float(settings['step_delay_sin_neck'])

    

    # Define parameters
    #main_period = 2.0   # global variable: this is the period of the first sinusoidal functio
    # Frequency of the first sinusoidal function
    main_frequency = 1 /  main_period

    amplitude1 = 1.0  # amplitude of the first sinusoidal function
    amplitude2 = 1.0  # amplitude of the second sinusoidal function
    amplitude3 = 1.0  # amplitude of the third sinusoidal function
    amplitude4 = 1.0  # amplitude of the third sinusoidal function
    
    #  lower leg sinusoid2 delay in seconds, this is a global variable
    # step_delay_sin_lower_leg = main_period/16


    # Calculate the frequency of the second sinusoidal function
    if step_delay_sin_lower_leg == main_period/2:
        frequency2 = main_frequency *2  # Set a default frequency if delay equals period
    else:
        frequency2 =( 1 / (main_period/2 - step_delay_sin_lower_leg))/2


        # Upper leg sinusoid3 delay in seconds, this is a global variable
    #step_delay_sin_upper_leg = main_period/8


    # Calculate the frequency of the second sinusoidal function
    if step_delay_sin_upper_leg == main_period/2:
        frequency3 = main_frequency *2  # Set a default frequency if delay equals period
    else:
        frequency3 =( 1 / (main_period/2 - step_delay_sin_upper_leg))/2

    # Calculate the frequency of the second sinusoidal function
    if step_delay_sin_upper_leg == main_period/2:
        frequency4 = main_frequency *2  # Set a default frequency if delay equals period
    else:
        frequency4 =( 1 / (main_period/2 - step_delay_sin_anckle))/2


    # Time values
    
    #duration = num_cycles* Period  # Duration of the trajectory (in seconds)
    duration = main_period/2  # Duration of the trajectory (in seconds)
    num_samples = int(duration/ sampling_time) # Number of samples
    #num_samples = 1000
    print("num_samples:",num_samples )
    phase = 0.0 # np.pi / 2   # Phase shift (in radians)


    # Time array
    t = np.linspace(0, duration, num_samples)

    # Create sinusoidal data for both functions without delay
    sinusoid11 = amplitude1 * np.sin(2 * np.pi * main_frequency * t*CM_frequency_factor)
    sinusoid21 = amplitude2 * np.sin(2 * np.pi * frequency2 * t)
    sinusoid31 = amplitude3 * np.sin(2 * np.pi * frequency3 * t)
    sinusoid41 = amplitude4 * np.sin(2 * np.pi * frequency4 * t)


    #****** First Half of step ******
    # Calculate delay in samples
    delay_samples_sinusoid2 = int(step_delay_sin_lower_leg / (t[1] - t[0]))

    # Apply delay to the second sinusoid
    sinusoid21_delayed = np.roll(sinusoid21, delay_samples_sinusoid2)

    # Adjust the delayed sinusoid to start from zero
    sinusoid21_delayed[:delay_samples_sinusoid2] = 0




    #****** Second Half of step ******
    # Calculate delay in samples
    delay_samples_sinusoid3 = int(step_delay_sin_upper_leg / (t[1] - t[0]))

    # Apply delay to the second sinusoid
    sinusoid31_delayed = np.roll(sinusoid31, delay_samples_sinusoid3)

    # Adjust the delayed sinusoid to start from zero
    sinusoid31_delayed[:delay_samples_sinusoid3] = 0

    #****** Second Half of step ******
    # Calculate delay in samples
    delay_samples_sinusoid4 = int(step_delay_sin_anckle / (t[1] - t[0]))

    # Apply delay to the second sinusoid
    sinusoid41_delayed = np.roll(sinusoid41, delay_samples_sinusoid4)

    # Adjust the delayed sinusoid to start from zero
    sinusoid41_delayed[:delay_samples_sinusoid4] = 0



    # **** Concatenate both half_size_step *****

    if half_size_step == 2:
        sinusoid1 = np.concatenate((sinusoid11,-sinusoid11))
        sinusoid2 = np.concatenate((sinusoid21_delayed,-sinusoid21_delayed))
        sinusoid3 = np.concatenate((sinusoid31_delayed,-sinusoid31_delayed))
        sinusoid4 = np.concatenate((sinusoid41_delayed,-sinusoid41_delayed))
    # **** left half step *****

    if half_size_step == 1:
        sinusoid1 = sinusoid11
        sinusoid2 = sinusoid21_delayed
        sinusoid3 = sinusoid31_delayed
        sinusoid4 = sinusoid41_delayed
        
    total_duration = cycles * main_period       # Duration of the trajectory (in seconds)
    total_num_samples = cycles * 2* num_samples   # Number of samples


    #**** Repeat the steps n times ****
    # Repeat the array n times
    sinusoid1 = np.tile(sinusoid1, cycles)
    sinusoid2 = np.tile(sinusoid2, cycles)
    sinusoid3 = np.tile(sinusoid3, cycles)
    sinusoid4 = np.tile(sinusoid4, cycles)

    duration2 = cycles * half_size_step*duration
    num_samples2 = int(cycles * half_size_step*num_samples)
    t2 = np.linspace(0, duration2, num_samples2)

    zeros_trajectory = np.zeros(num_samples2)
    # Stack sinusoidal data into columns of a global array
    sin_trajectory = np.column_stack(( sinusoid2, sinusoid3, sinusoid1, zeros_trajectory, sinusoid2, sinusoid3, sinusoid1, zeros_trajectory, sinusoid1, sinusoid4, sinusoid4))

    

    #Plotting the sinusoidal functions from the global array
    # plt.figure(figsize=(10, 6))
    # plt.plot(t2, sin_trajectory[:, 0], label=f'Sinusoid M3 - {frequency2} Hz')
    # plt.plot(t2, sin_trajectory[:, 1], label=f'Sinusoid M2 - {frequency3} Hz')
    # plt.plot(t2, sin_trajectory[:, 2], label=f'Sinusoid M6 - {main_frequency} Hz')
    # plt.plot(t2, sin_trajectory[:, 3], label=f'Sinusoid M9 - {0} Hz')
    # plt.plot(t2, sin_trajectory[:, 4], label=f'Sinusoid M5 - {frequency2} Hz')
    # plt.plot(t2, sin_trajectory[:, 5], label=f'Sinusoid M4 - {frequency3} Hz')
    # plt.plot(t2, sin_trajectory[:, 6], label=f'Sinusoid M7 - {main_frequency} Hz')
    # plt.plot(t2, sin_trajectory[:, 7], label=f'Sinusoid M8 - {0} Hz')
    # plt.plot(t2, sin_trajectory[:, 8], label=f'Sinusoid M10 - {main_frequency} Hz')
    # plt.plot(t2, sin_trajectory[:, 9], label=f'Sinusoid M12 - {frequency4} Hz')
    # plt.plot(t2, sin_trajectory[:, 10], label=f'Sinusoid M11 - {frequency4} Hz')


    # plt.title('Four Sinusoidal Functions with delay  Shift')
    # plt.xlabel('Time (seconds)')
    # plt.ylabel('amplitude')
    # plt.legend()
    # plt.grid(True)
    # plt.show()





    print("sin_trajectory:",sin_trajectory )
    idx_theta = 0     
 

# This function enters and leaves every Period, However, it CREATES its own trajectory, that is: it does not call  trajectory_ang_space(positions)
#It sends position directly to de motors  
def sinusolidal_step(amplitudes):
    global debug
    global num_cycles
    global local_cycles
    global HOME_POSITIONS
    update_home_positions  # why do I have to do this?
    
    global sin_trajectory
    global idx_theta #global index for the trajectory
    global direction
    global desired_ang_position #global variable containing the instantaneous desired position of all the robot joints
    global command

    
    
    if local_cycles < num_cycles:
        

             #At every step(Period)  a new point (row) of the trajectory is executed
            theta = sin_trajectory[idx_theta] #selects row by row.. Every row has 11 angles corresponding to the 11 motors
            #print("\r idx_theta",idx_theta)   
            #print ("\r theta",theta)  
            
            new_ang_positions = []
            for idx, ini_angle in enumerate (HOME_POSITIONS):
                joint_angle_amplitude = MOTOR_SIGN[idx]
                joint_angle = theta[idx]
                new_ang_positions.append(ini_angle  + joint_angle_amplitude*joint_angle) #add (trajectory angles * amplitudes) to every motor angle position in HOME_POSITIONS
            #set_ang_position(new_ang_positions)
            desired_ang_position = new_ang_positions #By doing this, the main program  take this "desired_ang_position" and will move to joints by calling the function: set_ang_position(desired_ang_position)
            
            idx_theta += 1
            if idx_theta >= len(sin_trajectory):
                idx_theta = 0 #reset the global trajectory index
                local_cycles+=1         

                print("\r local_cycles",local_cycles)      
    else:
        local_cycles = 0
        idx_theta = 0
        command = ""   # VERY IMPORTANT, to finish the movement!
            

 #This function may not be neccesary
def jogging_step(amplitudes):
    global debug
    global num_cycles
    global local_cycles
    global HOME_POSITIONS
    update_home_positions  # why do I have to do this?
    
    global sin_trajectory
    global idx_theta
    global direction
    global desired_ang_position
    global command

    
    
    if local_cycles < num_cycles:
        

             
            theta = sin_trajectory[idx_theta] #selects row by row.. Every row has 8 angles corresponding to the 8 motors
            #print("\r idx_theta",idx_theta)   
            #print ("\r theta",theta)  
            
            new_ang_positions = []
            for idx, ini_angle in enumerate (HOME_POSITIONS):
                joint_angle_amplitude = MOTOR_SIGN[idx]
                joint_angle = theta[idx]
                new_ang_positions.append(ini_angle  + joint_angle_amplitude*joint_angle) #add (trajectory angles * amplitudes) to every motor angle position in HOME_POSITIONS
            #set_ang_position(new_ang_positions)
            desired_ang_position = new_ang_positions
            
            idx_theta += 1
            if idx_theta >= len(sin_trajectory):
                idx_theta = 0
                local_cycles+=1         

                print("\r local_cycles",local_cycles)      
    else:
        local_cycles = 0
        idx_theta = 0
        command = ""   # VERY IMPORTANT, to finish the movement! 
                                     
def step():
    global num_cycles
    global HOME_POSITIONS
    update_home_positions # why do I have to do this?
    
     # Generate thetas values
    delta_angle = np.pi/4.0
    resolution = 100
    thetas = np.linspace(-delta_angle,delta_angle , resolution)  # 100 points from 0 to delta_angle . Importante empezar desde ZERO, para evitar un brinco a la primera posicion!!
       
    #first_step(delta_angle)
    
    
    

    #np_positions = np.array(HOME_POSITIONS)
    
   
    direction = 1  # Direction of traversal (1 for increasing, -1 for decreasing)
    idx_theta = 0
    Millis_ant = 0
    Period = 100 
    local_cycles = 0
    
    while local_cycles < num_cycles:
        
        CurrentMillis = millis()
        if (CurrentMillis - Millis_ant) >= Period:  # Entra cada "Period" segundos" 
            Millis_ant = CurrentMillis
                               
            theta = thetas[idx_theta] 
            
            new_ang_positions = []
            for idx, angle in enumerate (HOME_POSITIONS):
                sign = MOTOR_SIGN[idx]
                new_ang_positions.append(angle  + abs(sign)*theta) #add theta to every motor angle position in HOME_POSITIONS
            set_ang_position(new_ang_positions)
            
            idx_theta += direction
            if idx_theta >= len(thetas):
                direction = -1  # Reverse direction
                idx_theta = len(thetas) - 2  # Start from the second-to-last element
            elif idx_theta < 0:
                direction = 1  # Reverse direction
                idx_theta = 1  # Start from the second element
                local_cycles+=1 

    local_cycles = 0
    debug = 1
    display_XYZ_positions()
 
def condini():
    global settings
    global main_period
    global half_size_step

    print("\r ******************************* condini ***************************************\n")
    #Update the settings from the file body_settings.txt
    #settings = read_settings(file_path)  #this is not neccesary because update_home_positions()  does it
    update_home_positions()
    
    # Assign dictionary values to individual variables
    main_period = settings['main_period']
    half_size_step = settings['half_size_step']

    
    print("\n ********************** Initial Settings Dictionary: ****************************\n")
    #print(settings)
    for key, value in settings.items():
        print(f"{key}: {value}")
    
    
    
    
    


def robot_control_logic():
    global command
    global new_ang_position
    global old_ang_position
    global old_XYZ_position
    global desired_ang_position
    global trajectory_finished
    global salir
    global debug
    
    global step_upper_leg_ang
    global step_lower_leg_ang 
    global step_CM_displacement_ang 
    global main_period
    global step_delay_sin_lower_leg 
    global step_delay_sin_upper_leg 
    global half_size_step 
    global step_turn_ang 
    global STEP_amplitudes

    global Millis_ant

    global count
    global Period


    CurrentMillis = millis()
    if (CurrentMillis - Millis_ant) >= Period:  # Entra cada "Period" segundos" 
        Millis_ant = CurrentMillis
        
            #every 10 mSeg
        read_variables()  #this updates old_ang_position & old_XYZ_position
        if not trajectory_finished:
            
            execute_trajectory()
        else:    
            set_ang_position(desired_ang_position) #desired_ang_position has the desired instantaneous angles of the joints, in radians
            
        count+=1
            #every 200 mSeg
        if count >= 20:
            count = 0
            publish_variables()
            display_RPY_angles()   
            
            
        if command == "home":
            print("\r Going home")
            debug = 1
            home()
            command ="" 
        if command == "reset":
            print("\r Going reset")
            debug = 1
            reset()
            command ="" 
            
        if command == "up_down":
            #print("\r Going up_down")
            debug = 1
            #home() #send it to home first to avoid a jump to the middle position
            up_down()#uses the global variable num_cycles
            
            #command =""  DO NOT pu this line. command needs to be unchangend to let the program enters here until the end of the cycles
            
        if command == "Forward_step":
            #print("\r Going step")
            debug = 1
            #home() #send it to home first to avoid a jump to the middle position
            #step()#uses the global variable num_cycles
                

            # = [step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang]
            #STEP_amplitudes = np.radians(STEP_amplitudes)
            sinusolidal_step(STEP_amplitudes)
            #home() #send it to home last to return to the middle position
            #command =""  DO NOT pu this line. command needs to be unchangend to let the program enters here until the end of the cycles
            
        if command == "Jogging_step":
            #print("\r Going step")
            debug = 1
            #home() #send it to home first to avoid a jump to the middle position
            #step()#uses the global variable num_cycles
                

            # = [step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang]
            #STEP_amplitudes = np.radians(STEP_amplitudes)
            sinusolidal_step(Jogging_amplitudes)
            #home() #send it to home last to return to the middle position
            #command =""  DO NOT pu this line. command needs to be unchangend to let the program enters here until the end of the cycles  
            
        if command == "side_step":
            #print("\r Going side_step")
            debug = 1
            #home() #send it to home first to avoid a jump to the middle position
            #step()#uses the global variable num_cycles
            sinusolidal_step(SIDE_STEP_amplitudes)
            #home() #send it to home last to return to the middle position
            #command =""  DO NOT pu this line. command needs to be unchangend to let the program enters here until the end of the cycles                
            
        elif command == "positionXYZ":
            debug = 1
            set_XYZ_position(new_XYZ_position)
            command =""
        elif command == "TrajectoryXYZ":
            debug = 1
            #trajectory_XYZ(new_XYZ_position)
            trajectory_ang_XYZ(new_XYZ_position)
            command =""   
        elif command == "monitor_angles":
            debug = 2
            display_ang_positions()
            command =""
        elif command == "monitor_XYZ":
            debug = 1
            display_XYZ_positions()
            command =""
        elif command == "monitor_RPY":
            debug = 1
            display_RPY_angles()
            command =""
        elif command == "monitor_torques":
            debug = 1
            display_ang_torques()  
            command =""                
        elif command == "monitor_currents":
            debug = 1
            display_ang_currents()   
            command =""

def robot_control():
    global salir
    global Millis_ant
    global trajectory_finished
    global count
    global Period



    condini()
                
    Period = 20   
    Millis_ant = 0
    
    count = 0        
    salir = False
    
    trajectory_finished = True
    read_variables()#this updates old_ang_position & old_XYZ_position
    publish_variables()  
    setup_sinusolidal_step(1)
    

    print("\n *************  Starting control. Press ctrl+C to quit ************ \n")
    
    while not(salir):
        robot_control_logic()



                                     
# def robot_control_BAK(): #this is my original robot_control function

#     global command
#     global new_ang_position
#     global old_ang_position
#     global old_XYZ_position
#     global desired_ang_position
#     global  trajectory_finished
#     global salir
#     global debug
    
#     global step_upper_leg_ang
#     global step_lower_leg_ang 
#     global step_CM_displacement_ang 
#     global main_period
#     global step_delay_sin_lower_leg 
#     global step_delay_sin_upper_leg 
#     global half_size_step 
#     global step_turn_ang 
#     global STEP_amplitudes
    
#     print("Starting control. Press ctrl+C to quit.")
   
   

#     condini()
                
                
#     Millis_ant = 0
#     Period = 10
     
#     count = 0        
#     salir = False
    
#     trajectory_finished = True
#     read_variables()#this updates old_ang_position & old_XYZ_position
#     publish_variables()  
    
#     while  not(salir):
  
#         CurrentMillis = millis()
#         if (CurrentMillis - Millis_ant) >= Period:  # Entra cada "Period" segundos" 
#             Millis_ant = CurrentMillis
            
#                 #every 10 mSeg
#             read_variables()  #this updates old_ang_position & old_XYZ_position
#             if not trajectory_finished:
#                 execute_trajectory()
#             else:    
#                 set_ang_position(desired_ang_position)
                
#             count+=1
#                 #every 100 mSeg
#             if count >= 10:
#                 count = 0
#                 publish_variables()
#                 display_RPY_angles()   
                
             
#             if command == "home":
#                 print("\r Going home")
#                 debug = 1
#                 home()
#                 command ="" 
#             if command == "reset":
#                 print("\r Going reset")
#                 debug = 1
#                 reset()
#                 command ="" 
                
#             if command == "up_down":
#                 #print("\r Going up_down")
#                 debug = 1
#                 #home() #send it to home first to avoid a jump to the middle position
#                 up_down()#uses the global variable num_cycles
                
#                 #command =""  DO NOT pu this line. command needs to be unchangend to let the program enters here until the end of the cycles
                
#             if command == "Forward_step":
#                 #print("\r Going step")
#                 debug = 1
#                 #home() #send it to home first to avoid a jump to the middle position
#                 #step()#uses the global variable num_cycles
                    

#                 #STEP_amplitudes = [step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang]
#                 #STEP_amplitudes = np.radians(STEP_amplitudes)
#                 sinusolidal_step(STEP_amplitudes)
#                 #home() #send it to home last to return to the middle position
#                 #command =""  DO NOT pu this line. command needs to be unchangend to let the program enters here until the end of the cycles
                
#             if command == "Jogging_step":
#                 #print("\r Going step")
#                 debug = 1
#                 #home() #send it to home first to avoid a jump to the middle position
#                 #step()#uses the global variable num_cycles
                    

#                 #STEP_amplitudes = [step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang]
#                 #STEP_amplitudes = np.radians(STEP_amplitudes)
#                 sinusolidal_step(Jogging_amplitudes)
#                 #home() #send it to home last to return to the middle position
#                 #command =""  DO NOT pu this line. command needs to be unchangend to let the program enters here until the end of the cycles  
                
#             if command == "side_step":
#                 #print("\r Going side_step")
#                 debug = 1
#                 #home() #send it to home first to avoid a jump to the middle position
#                 #step()#uses the global variable num_cycles
#                 sinusolidal_step(SIDE_STEP_amplitudes)
#                 #home() #send it to home last to return to the middle position
#                 #command =""  DO NOT pu this line. command needs to be unchangend to let the program enters here until the end of the cycles                
                
#             elif command == "positionXYZ":
#                 debug = 1
#                 set_XYZ_position(new_XYZ_position)
#                 command =""
#             elif command == "TrajectoryXYZ":
#                 debug = 1
#                 #trajectory_XYZ(new_XYZ_position)
#                 trajectory_ang_XYZ(new_XYZ_position)
#                 command =""   
#             elif command == "monitor_angles":
#                 debug = 2
#                 display_ang_positions()
#                 command =""
#             elif command == "monitor_XYZ":
#                 debug = 1
#                 display_XYZ_positions()
#                 command =""
#             elif command == "monitor_RPY":
#                 debug = 1
#                 display_RPY_angles()
#                 command =""
#             elif command == "monitor_torques":
#                 debug = 1
#                 display_ang_torques()  
#                 command =""                
#             elif command == "monitor_currents":
#                 debug = 1
#                 display_ang_currents()   
#                 command =""     

     

        
            

#  #####################################################################
#   #  main program 
#  ####################################################################    


if __name__ == '__main__':
    

# to use additional motors, simply add another with block
# remember to give each motor a different log name!

                                                      
    print("\n +++++++++++++++ Initialization:  ++++++++++++++  \n")   
    
##########################################################
    # MQTT setup

    print("\r Setting Up MQTT \n")

    setup_mqtt()


     
 
                            
    #joints_labels = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"] 
      
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
                                                    #Add TmotorCanControl motor objects to a list called motors
                                                    motors.extend([dev1,dev2,dev3,dev4,dev5,dev6,dev7,dev8,dev9,dev10,dev11])
                                                    #Create a dictionary with the joint labels and the T-motor objects
                                                    T_motors_dict = dictionary = dict(zip(joints_labels, motors))
                                                    #print(T_motors_dict)   #prints many information about the motors

                                                    print(VERSION)
                                                    print("\n +++++++++++++++ Init motors:  ++++++++++++++  \n")

                                                    debug=1
                                                    init_motors()
                                                    print("\n +++++++++++++++ Init PD controllers:  ++++++++++++++  \n")
                                                    init_gains()
                                                    
                                                    display_ang_positions()
                                                    #home()  #The supports put it in home
                                                    #print("Going home")
                                                    #set_ang_position(HOME_POSITIONS)                                      
                                                    robot_control()
                                                    if salir :
                                                            sys.exit()
                                            
            
     #except:
