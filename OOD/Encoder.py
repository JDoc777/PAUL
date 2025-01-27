import rwSerial
import json
import time
import queue
import serial
import threading
import sys
from queue import Empty
import traceback
import Velocity
# Encoder state setup



class Encoder:
   
    
    def __init__(self):
        self.encoder_state = {
        'FL': {'last_encoder_pos': 0, 'last_time': time.time()},
        'FR': {'last_encoder_pos': 0, 'last_time': time.time()},
        'BL': {'last_encoder_pos': 0, 'last_time': time.time()},
        'BR': {'last_encoder_pos': 0, 'last_time': time.time()}
        }
        self.prev_pos = 0
        self.delta_t = 0
        self.current_encoder_pos = 0
        
    def update_encoder_position(self, wheel, current_encoder_pos):
        
            
            # Get the current timestamp
            current_time = time.time()

            # Calculate delta_t (time difference)
            delta_t = current_time - self.encoder_state[wheel]['last_time']

            # Store the previous position
            prev_pos = self.encoder_state[wheel]['last_encoder_pos']

            # Update the last encoder position and timestamp for the wheel
            self.encoder_state[wheel]['last_encoder_pos'] = current_encoder_pos
            self.encoder_state[wheel]['last_time'] = current_time
            
            self.prev_pos = 5
            self.delta_t = 6
            self.current_encoder_pos = 7
            
            
            return prev_pos, current_encoder_pos, delta_t
    def get_prev_pos(self):
        return self.prev_pos
            
    def choose_wheel(self,wheel_dict):
        while True:
        
        
            FL = self.update_encoder_position('FL',wheel_dict['FL'])
            BL = self.update_encoder_position('BL',wheel_dict['BL'])
            FR = self.update_encoder_position('FR',wheel_dict['FR'])
            BR = self.update_encoder_position('BR',wheel_dict['BR'])
            
            
            
        return FL, BL, FR, BR
        
    
    def get_delta_t(self):
        return self.delta_t
    def get_current_encoder_pos(self):
        return self.current_encoder_pos
        
