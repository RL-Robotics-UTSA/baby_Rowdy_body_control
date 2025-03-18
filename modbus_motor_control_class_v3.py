
#Rowdy Robot project
#UTSA
#Sergio Montufar

#v 3.0   July 26 2024
#  ChatGPT assisted script:

#This script now supports two MODBUS slaves (right_motor and left_motor) on different serial ports and performs
#the necessary read and write operations for both. Adjust the register addresses and values as needed .

#Two Modbus Instruments: Configured right_motor to use /dev/ttyUSB0 and left_motor to use /dev/ttyUSB1.
#Serial Configuration: Configured both right_motor and left_motor.
#write_registers and read_registers Functions: Modified to take an extra parameter motor, which specifies which MODBUS instrument to use.
#Main Loop: Calls write_registers and read_registers for both right_motor and left_motor every second, allowing for other tasks to be performed during the wait period.

#The functionality is encapsulated within a MotorController class.
#The class methods write_registers and read_registers handle writing to and reading from the Modbus registers.
#The __init__ method initializes the Modbus instrument and configures the serial connection.
#The if __name__ == "__main__": block is used for testing purposes, demonstrating how to use the class within the same file.
#Another script (another_script.py) shows how to import and use the MotorController class to interact with the motors.

#Error Handling in __init__ Method:

#The __init__ method now includes a try-except block to handle potential serial.SerialException errors that might occur during the connection to the motor. If an error occurs, it logs the error and sets self.motor to None.
#Validation in write_registers and read_registers Methods:

#Example Usage:
#before performing any operations, both methods check if self.motor is None. If it is, an error message is logged and the method returns early, avoiding any further operations on a non-existent motor.


    #In the __main__ block, before attempting to write or read registers, the script checks if the motor is connected by verifying that self.motor is not None.
    #This ensures that any connection errors are handled gracefully.

#To avoid displaying DEBUG messages in your logging output, you need to set the logging level to a higher threshold. 
# In Python's logging module, you can set the logging level to INFO, WARNING, ERROR, or CRITICAL to filter out debug messages.
# log.setLevel(logging.INFO) sets the logging level to INFO, which will filter out any DEBUG level messages.

import minimalmodbus
import serial
import logging

# Enable logging
logging.basicConfig()
log = logging.getLogger()
#log.setLevel(logging.DEBUG)
log.setLevel(logging.INFO)  # Set to INFO to avoid DEBUG messages

class MODBUS_MotorController:
    def __init__(self, port, address):
        self.settings = {
            'speed': 0,
            'maximum_speed': 100,
            'minimum_speed': 0,
            'acceleration': 5,
            'deceleration': 5,
            'enable': 1
        }
        self.speed_converted = 0

        try:
            self.motor = minimalmodbus.Instrument(port, address)
            self.motor.serial.baudrate = 9600
            self.motor.serial.bytesize = 8
            self.motor.serial.parity = serial.PARITY_NONE
            self.motor.serial.stopbits = 1
            self.motor.serial.timeout = 1
            log.info(f"Connected to motor at {port} with address {address}")
        except serial.SerialException as e:
            print(f"Error connecting to motor at {port} with address {address}: {e}")
            self.motor = None
        except Exception as e:
            print(f"Unexpected error when connecting to motor at {port} with address {address}: {e}")
            self.motor = None

    def display_settings(self):
        """Method to display the current settings."""
        for key, value in self.settings.items():
            print(f"{key}: {value}")

    def get_settings(self):
        """Method to return the current settings as a dictionary."""
        return self.settings

    def update_settings(self, new_settings):
        """Method to update the current settings."""
        for key, value in new_settings.items():
            if key in self.settings:
                self.settings[key] = value
            else:
                print(f"Warning: '{key}' is not a valid setting.")
        print("Settings updated.")

    @staticmethod
    def signed_to_unsigned(value):
        if value < 0:
            return value + 0x10000
        return value

    @staticmethod
    def unsigned_to_signed(value):
        if value >= 0x8000:
            return value - 0x10000
        return value

    def write_registers(self):
        if self.motor is None:
            print("Motor is not connected.")
            return None
        try:
            if not (0 <= self.settings['maximum_speed'] <= 100):
                print("Speed limit out of range (0-100)")
                return
            if not (-100 <= self.settings['speed'] <= 100):
                print("Speed out of range (-100 to 100)")
                return
            if not (0 <= self.settings['minimum_speed'] <= 100):
                print("Minimum speed out of range (0-100)")
                return
            if not (0 <= self.settings['acceleration'] <= 100):
                print("Acceleration out of range (0-100)")
                return
            if not (0 <= self.settings['deceleration'] <= 100):
                print("Deceleration out of range (0-100)")
                return

            self.speed_converted = int(self.settings['speed'] * 1023 / 100)

            print(f"Writing converted speed to register 0: {self.speed_converted}")
            self.motor.write_register(0, self.signed_to_unsigned(self.speed_converted))

            print(f"Writing maximum speed to register 1: {self.settings['maximum_speed']}%")
            self.motor.write_register(1, self.signed_to_unsigned(self.settings['maximum_speed']))

            print(f"Writing minimum speed to register 2: {self.settings['minimum_speed']}")
            self.motor.write_register(2, self.signed_to_unsigned(self.settings['minimum_speed']))

            print(f"Writing acceleration to register 3: {self.settings['acceleration']}")
            self.motor.write_register(3, self.signed_to_unsigned(self.settings['acceleration']))

            print(f"Writing deceleration to register 4: {self.settings['deceleration']}")
            self.motor.write_register(4, self.signed_to_unsigned(self.settings['deceleration']))

            print(f"Writing enable to register 5: {self.settings['enable']}")
            self.motor.write_register(5, self.signed_to_unsigned(self.settings['enable']))

        except Exception as e:
            print(f"Error writing to registers on {self.motor}: {e}")

    def read_registers(self):
        if self.motor is None:
            print("Motor is not connected.")
            return None
        try:
            speed = self.motor.read_register(0)
            maximum_speed = self.motor.read_register(1)
            minimum_speed = self.motor.read_register(2)
            acceleration = self.motor.read_register(3)
            deceleration = self.motor.read_register(4)
            enable = self.motor.read_register(5)

            speed_signed = self.unsigned_to_signed(speed)
            speed_limit_signed = self.unsigned_to_signed(maximum_speed)
            minimum_speed_signed = self.unsigned_to_signed(minimum_speed)
            acceleration_signed = self.unsigned_to_signed(acceleration)
            deceleration_signed = self.unsigned_to_signed(deceleration)
            enable_signed = self.unsigned_to_signed(enable)

            speed_percent = (speed_signed * 100 / 1023)

            # log.debug(f"Read from register 0 (speed): {speed_percent:.2f}%")
            # log.debug(f"Read from register 0 (speed): {speed_signed}")
            # log.debug(f"Read from register 1 (maximum_speed): {speed_limit_signed}%")
            # log.debug(f"Read from register 2 (minimum_speed): {minimum_speed_signed}")
            # log.debug(f"Read from register 3 (acceleration): {acceleration_signed}")
            # log.debug(f"Read from register 4 (deceleration): {deceleration_signed}")
            # log.debug(f"Read from register 5 (enable): {enable_signed}")
            
            # print(f"Read from register 0 (speed): {speed_percent:.2f}%")
            # print(f"Read from register 0 (speed): {speed_signed}")
            # print(f"Read from register 1 (maximum_speed): {speed_limit_signed}%")
            # print(f"Read from register 2 (minimum_speed): {minimum_speed_signed}")
            # print(f"Read from register 3 (acceleration): {acceleration_signed}")
            # print(f"Read from register 4 (deceleration): {deceleration_signed}")
            # print(f"Read from register 5 (enable): {enable_signed}")
            
            

            # Update settings dictionary
            self.settings['speed'] = speed_signed
            self.settings['maximum_speed'] = speed_limit_signed
            self.settings['minimum_speed'] = minimum_speed_signed
            self.settings['acceleration'] = acceleration_signed
            self.settings['deceleration'] = deceleration_signed
            self.settings['enable'] = enable_signed

            return self.settings

        except Exception as e:
            print(f"Error reading from registers on {self.motor}: {e}")
            return None

# Example usage
if __name__ == "__main__":
    # Create an instance of the motor controller
    motor_controller = MODBUS_MotorController("/dev/ttyUSB0", 1)

    # Display current settings
    motor_controller.display_settings()

    # Update settings
    new_settings = {
        'speed': 50,
        'maximum_speed': 90,
        'acceleration': 10,
        'deceleration': 10,
        'enable': 1
    }
    motor_controller.update_settings(new_settings)
    motor_controller.display_settings()

    # Write to registers
    motor_controller.write_registers()

    # Read from registers and display updated settings
    updated_settings = motor_controller.read_registers()
    print("Updated Settings:", updated_settings)
