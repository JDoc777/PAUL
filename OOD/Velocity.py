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
import rwSerial
import math
import numpy as np

class Velocity():

    
    def __init__(self):
        self.encoder = Encoder.Encoder()
        self.prev_pos, self.current_encoder_pos, self.delta_t = self.encoder.get_prev_pos(),self.encoder.get_current_encoder_pos(),5
        
    def calculate_velocity(self,prev_pos, current_encoder_pos, delta_t):
    
    # Constants
        radius = 0.04  # Radius in meters (40 mm)
        pulses_per_revolution = 111.25  # Pulses per revolution (PPR)

    # Calculate the change in encoder position
        delta_n = self.current_encoder_pos - self.prev_pos
        
    # Calculate velocity using the formula
        if self.delta_t > 0:
            velocity = (2 * math.pi * radius * delta_n) / (pulses_per_revolution * self.delta_t)
            velocity = velocity * 25
           
            return velocity
    
    
        
    def test_print(self):
        print(f"Previous Position: {self.prev_pos}")
        print(f"Current Position: {self.current_encoder_pos}")
        print(f"Delta T: {self.delta_t}")

