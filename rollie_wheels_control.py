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
    - adjust direct_kinematics and inverse_kinematics to acomplish that
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
Correct direct_kinematics and inverse_kinematic fuctions 
INITIAL POSITION of the robot MUST be Completely vertical!! (Requires the support)
    - direct_kinematics and inverse_kinematics to acomplish that
    
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
    
V3
July 26, 2024
    - Incorporate wheels control via MQTT from UI   
 
 
 V3
August 6, 2024
    - Decouple the wheels control from the legs control
    - A new program is generated (rollie_wheels_control v1.0 ) to control only the wheels. This is beause the wheels control slowers the legs control loop
    
To do..   
    - adition to sequential home roitine, one leg a a time
    - Equilibrium functions implemented
    
    
'''

VERSION = "rollie_wheels_control v1.0"


import sys
from time import sleep

import time

# Import the motor control class


from modbus_motor_control_class_v3 import MODBUS_MotorController

##########################################################
# Wheels DC Motor variables
right_motor_dict = {}
left_motor_dict = {}
    
# Define the serial ports and Modbus slave addresses
right_motor_port = '/dev/ttyUSB1'
left_motor_port = '/dev/ttyUSB2'
right_motor_address = 1
left_motor_address = 1

# Create instances of the MotorController class
right_motor = MODBUS_MotorController(right_motor_port, right_motor_address)
left_motor = MODBUS_MotorController(left_motor_port, left_motor_address)


speed = 100
speed_limit = 100
minimum_speed = 0
acceleration = 5
deceleration = 5
enable = 1



debug = False
command = ""

#Wheel speed variables
Speed = 0
Turn = 0
Enable_Wheels = False


##########################################################
# MQTT setup
import paho.mqtt.client as mqtt
import ssl
import json


############################################################################
# MQTT Parameters & Functions

serverAddress = "localhost"
#serverAddress = "127.0.0.1"
#serverAddress = "rollie-body-pi"
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
        
        mqttClient.subscribe([  ("body/control/command", 1),("body/control/monitor", 1), ("body/control/exit", 1)])

        print("subscribed")
        print("Connected to Mosquitto result code "+str(rc))
 # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    #AWS
    # client.subscribe([("$aws/things/raspberryPi4/shadow/update", 1), ("$aws/things/raspberryPi4/shadow/get", 1)])
    
def on_message(client, userdata, message):
    
    global debug
    global salir
    global command

 
    global Speed
    global Turn
    global Enable_Wheels

    
    print("Message received: " + message.topic + " : " + str(message.payload))
    #print("topic :",message.topic)
    #print(type(message.topic))      
    
    #print("payload :",message.payload)
    #print(type(message.payload)) 
       
    m_decode=str(message.payload.decode("utf-8","ignore"))
    
    #print("data Received type",type(m_decode))
    print("data Received",m_decode)
    try:
        # Converting from JSON to dictionary object
        m_in = json.loads(m_decode)  # decode JSON data
        #print(type(m_in))
        #print(m_in)
        
        # Print each key-value pair from JSON response
        for key, value in m_in.items():
            print(key, ":", value)
            print('\n')
    except json.JSONDecodeError as e:
        print(f"JSON decoding error: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
    
     
                             
                  
    if  message.topic == 'body/control/command':       

        if 'Wheels' in m_decode:    
            print("body/control/command  Wheels recibido") 
            if 'desired' in m_decode:    
                Speed = int((m_in["Wheels"]["desired"]["Speed"]))
                Turn = int((m_in["Wheels"]["desired"]["Turn"]))
                Enable_Wheels = bool((m_in["Wheels"]["desired"]["Enable"]))
                
                print(" //// Wheels parameters: //// ")
                print("\r Speed = ", Speed)
                print("\r Turn = ", Turn)
                print("\r Enable_Wheels = ", Enable_Wheels)
            
                if Enable_Wheels:
                    if  Turn == 0: #Go Forward or back depending on the value of speed (-100 - 100)
                        right_new_settings = {
                            'speed': int((m_in["Wheels"]["desired"]["Speed"])),
                            'enable': int((m_in["Wheels"]["desired"]["Enable"]))
                        }
                        left_new_settings = right_new_settings
                    else:
                        if Turn > 0: # Turn right
                            right_new_settings = {
                                'speed': -int(abs(Speed)),
                                'enable': int((m_in["Wheels"]["desired"]["Enable"]))
                            }
                            left_new_settings = {
                                'speed': int(abs(Speed)),
                                'enable': int((m_in["Wheels"]["desired"]["Enable"]))
                            }
                        else:
                            right_new_settings = {
                                'speed': int(abs(Speed)),
                                'enable': int((m_in["Wheels"]["desired"]["Enable"]))
                            }
                            left_new_settings = {
                                'speed': -int(abs(Speed)),
                                'enable': int((m_in["Wheels"]["desired"]["Enable"]))
                            }
                else:
                    
                    right_new_settings = {
                        'speed': 0,
                        'enable': 0
                    }
                    left_new_settings = right_new_settings
                    
                right_motor.update_settings(right_new_settings)
                left_motor.update_settings(left_new_settings)
            
                command = "wheels_control" 
                
            if 'parameters' in m_decode: 
                    # New settings to update
                new_settings = {
                    'maximum_speed': int((m_in["Wheels"]["parameters"]["Maximum_speed"])),
                    'maximum_speed': int((m_in["Wheels"]["parameters"]["Minimum_speed"])),                    
                    'acceleration': int((m_in["Wheels"]["parameters"]["Acceleration_step"])),
                    'deceleration': int((m_in["Wheels"]["parameters"]["Decceleration_step"]))
                }
                right_motor.update_settings(new_settings)
                left_motor.update_settings(new_settings)
            

            
  
            
    elif  message.topic == 'body/control/exit':       
        print("body/control/exit recibido") 
         
        salir = True   # al salir se va a Home!
    
# Function to publish message
def publish_message(client, topic, message):
    client.publish(topic, message)
    #print("Message published:", message)


  

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
     


def millis():
    return round(time.time() *1000)

   
def read_wheels_motors():
    global debug
    global right_motor_dict
    global left_motor_dict
    
    print(" **** Read Wheels variables ****\n") 

    # Read from registers if motor is connected
    if right_motor.motor is not None:
        right_motor_dict = right_motor.read_registers()  #Returns a dictionary with the values
        if debug ==1:
            if right_motor_dict:
                print("Right motor register values read:", right_motor_dict)
            
    if left_motor.motor is not None:
        left_motor_dict = left_motor.read_registers()  #Returns a dictionary with the values
        if debug ==1:
            if left_motor_dict:
                print("Left motor register values read:", left_motor_dict)


def wheels_control():
    
    # Write to registers
                # Write to registers and schedule read operation if motor is connected
    if right_motor.motor is not None:
        right_motor.write_registers()
        
    # Write to registers and schedule read operation if motor is connected
    if left_motor.motor is not None:
        left_motor.write_registers()
                                     
def skates_control():
    global command
    global salir
    global debug

    print("Starting control. Press ctrl+C to quit.")
           
    Millis_ant = 0
    Period = 100
     
    count = 0  
    
    salir = False
    
    
    while  not(salir):
  
        CurrentMillis = millis()
        if (CurrentMillis - Millis_ant) >= Period:  # enters every "Period" seconds" 
            Millis_ant = CurrentMillis
            
                
            count+=1

                
            if count >= 100:#every 10000 mSeg
                count = 0
                
                read_wheels_motors()  
                #publish_variables()
    
             

            if command == "wheels_control":
                print("\r Wheels comand control \n")
                debug = 1
                wheels_control()
                command ="" 

        

 #####################################################################
  #  main program 
 ####################################################################    

#mqttClient.loop_forever()# use this line if you don't want to write any further code. It blocks the code forever to check for data

if __name__ == '__main__':

    print("\r Rollie Rowdy wheels control program  Sergio Montufar UTSA")
    
 
    print(VERSION)
    print("\r +++++++++++++++ Initialization:  ++++++++++++++  ")


    
    
    
    debug=1

    
##########################################################
    # MQTT setup

    print("Setting Up MQTT")

    # Flag to indicate subscribe confirmation hasn't been printed yet.
    didPrintSubscribeMessage = False

    print('----------------------------------------')

    mqttClient = mqtt.Client()

    # Set up calling functions to mqttClient

    mqttClient.on_connect = on_connect  # attach function to callback
    mqttClient.on_message = on_message  # attach function to callback

    # Connect to the MQTT server  in the local LAN & loop forever.
    # CTRL-C will stop the program from running.
    print("server address is:", serverAddress)
    mqttClient.connect(serverAddress)

    print('----------------------------------------')
 
    #mqttClient.loop_forever()# use this line if you don't want to write any further code. It blocks the code forever to check for data

    # Start the MQTT client in a non-blocking thread
    mqttClient.loop_start() #use this line if you want to write any more code here to execute along the mqtt client

    # Write your main program here:
    # Main program continues to run concurrently

  
    #home()  #The supports put it in home
    #print("Going home")
    #set_ang_position(INI_POSITIONS)                                      
    skates_control()
    if salir :
            sys.exit()
                                            
            
     #except:
