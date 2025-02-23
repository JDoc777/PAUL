"""
This module is responsible for calculating the position of the robot based on the encoder data.

Global Variables included:
None

Usage:
This module can be imported as a module for the robot control system of PAUL.

Example for those who aren't experts at python:
from Encoder import Encoder

Classes:
Encoder is the class name but is a subclass of the Position class.
It is responsible for calculating the position of the robot based on the encoder data.


Dependecies ( Also known as Libraries):
json
time
queue
serial
threading
sys
Encoder
array
Velocity
PrintQueue
rwSerial
math
Position
"""

import json
import time
import queue
import serial
import threading
import sys
from queue import Empty
import traceback
import Encoder
import array
import Velocity
import PrintQueue
import rwSerial
import math
import Position as position

class Encoder(position.Position):
    def __init__(self,print_queue,rwSerial_object):
        super().__init__(print_queue, rwSerial_object)
        self.encoder_state = {
        'FL': {'last_encoder_pos': 0, 'last_time': time.time()},
        'FR': {'last_encoder_pos': 0, 'last_time': time.time()},
        'BL': {'last_encoder_pos': 0, 'last_time': time.time()},
        'BR': {'last_encoder_pos': 0, 'last_time': time.time()}
        }
    
        self.prev_pos = 0
        self.delta_t = 0
        self.current_encoder_pos = 0
        self.print_queue = print_queue
        self.rwSerial_object = rwSerial_object
        self.wheel_dict = {}  # Initializing the wheel_dict
       
                            
#     def update_encoder_position(self, wheel, wheel_dict, current_encoder_pos):
#         
#             
#             # Get the current timestamp
#             current_time = time.time()
# 
#             # Calculate delta_t (time difference)
#             self.delta_t = current_time - self.encoder_state[wheel]['last_time']
# 
#             # Store the previous position
#             self.prev_pos = self.encoder_state[wheel]['last_encoder_pos']
# 
#             # Update the last encoder position and timestamp for the wheel
#             self.encoder_state[wheel]['last_encoder_pos'] = current_encoder_pos
#             self.encoder_state[wheel]['last_time'] = current_time
#             
#             
#             
#             self.print_queue.add_message(f"""
# 
#             Previous Position : {self.prev_pos}
#             Delta T           : {self.delta_t}
#             Current Encoder Pos: {self.current_encoder_pos}
#             Wheel				: {wheel}
#             Wheel Dict          : {wheel_dict}
# """)
#             #time.sleep(0.05)
#             return self.prev_pos, self.current_encoder_pos, self.delta_t

    def receive_encoder_serial(self):
        while True:
            try:
                if self.rwSerial_object.serial_port.in_waiting > 0:
                    line = self.rwSerial_object.serial_port.readline().decode('utf-8', errors='ignore').strip()  # Decode with error handling
                    
                    if line:
                            #self.print_queue.add_message(f"Raw data: {line}")  # Print the raw incoming data
                        
                            try:
                            # Parse JSON data
                                data = json.loads(line)
                            
                            # Extract encoder positions for all wheels
                                encoderPos_FL = data['enco']['1']['position'] 
                                encoderPos_BL = data['enco']['2']['position'] 
                                encoderPos_FR = data['enco']['3']['position']
                                encoderPos_BR = data['enco']['4']['position']
                            
                                return encoderPos_FL, encoderPos_BL, encoderPos_FR, encoderPos_BR
                            
                            except json.JSONDecodeError:
                                self.print_queue.add_message("Error: Received invalid JSON data.")
                            except KeyError as e:
                                self.print_queue.add_message(f"Error: Missing key in JSON data - {e}")
                    else:
                            self.print_queue.add_message("No data received.")
                time.sleep(0.01)  # Reduced delay to prevent lag
            except Exception as e:
                    self.print_queue.add_message(f"Error while reading serial data: {e}")
                    traceback.print_exc()
                    
    def update_encoder_position(self, wheel, current_encoder_pos):
        
            
            # Get the current timestamp
            current_time = time.time()

            # Calculate delta_t (time difference)
            self.delta_t = current_time - self.encoder_state[wheel]['last_time']

            # Store the previous position
            self.prev_pos = self.encoder_state[wheel]['last_encoder_pos']

            # Update the last encoder position and timestamp for the wheel
            self.encoder_state[wheel]['last_encoder_pos'] = current_encoder_pos
            self.encoder_state[wheel]['last_time'] = current_time
            
            
         
            #time.sleep(0.05)
            return self.prev_pos, current_encoder_pos, self.delta_t
   
                    
    def choose_wheel(self):
        
        while True:
                
            encoderPos_FL, encoderPos_BL, encoderPos_FR, encoderPos_BR = self.receive_encoder_serial()
             
        

            # Update the encoder positions for each wheel
            FL = self.update_encoder_position('FL', encoderPos_FL)
            BL = self.update_encoder_position('BL', encoderPos_BL)
            FR = self.update_encoder_position('FR', encoderPos_FR)
            BR = self.update_encoder_position('BR', encoderPos_BR)
            
            #added dt average 
            dt0 = FL[2]
            dt1 = BL[2]
            dt2 = FR[2]
            dt3 = BR[2]
            
            dt = (dt0 + dt1 + dt2 + dt3) / 4
            
            
            self.wheel_dict = {
            'FL': FL[1],
            'BL': BL[1],
            'FR': FR[1],
            'BR': BR[1]
                    
                       }
            
#            self.print_queue.add_message(f"""
#             Current Position : {self.wheel_dict}
#                      Current Pos,Last Pos,Delta T
#             FL data: {FL[0]}, {FL[1]}, {FL[2]}
#             BL data: {BL[0]}, {BL[1]}, {BL[2]}
#             FR data: {FR[0]}, {FR[1]}, {FR[2]}
#             FL data: {BR[0]}, {BR[1]}, {BR[2]}
# """)
            # Returning the updated positions of all wheels
            return FL, BL, FR, BR, dt
            
        
    def get_prev_pos(self):
        return self.prev_pos
            
    def get_delta_t(self):
        return self.delta_t
    def get_current_encoder_pos(self):
        return self.current_encoder_pos
    
    def get_wheel_dict(self):
        return self.wheel_dict