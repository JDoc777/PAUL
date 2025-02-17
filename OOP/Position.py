"""
This module is responsible for calculating the position of the robot based on the velocity of the robot and the encoder data.

Global Variables included:
None

Usage:
This module can be imported as a module for the robot control system of PAUL.

Example for those who aren't experts at python:
from Position import Position

Classes:
Position is the class name but is a subclass of the Velocity class.
It is responsible for calculating the position of the robot based on the velocity of the robot and the encoder data.

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
import Position

class Position:
    def __init__(self, print_queue, rwSerial_instance):
        self.rwSerial_instance = rwSerial_instance
        self.print_queue = print_queue
        #self.velocity = Velocity.Velocity(print_queue, self.rwSerial_instance)
        #self.encoder = Encoder.Encoder(print_queue, self.rwSerial_instance)
        self.robot_position = {'x': 0.0, 'y': 0.0, 'angle': 0.0}
    
    
    def update_robot_position(self):
        while True:
#             Vx, Vy, Va = Velocity.calculate_velocity_components()
#             delta_t = self.encoder.choose_wheel()[4]
            
            # Current position
            Xold = self.robot_position['x']
            Yold = self.robot_position['y']
            theta_old = self.robot_position['angle']

            # Calculate new position
            Xnew = Xold + (Vx * math.cos(theta_old) - Vy * math.sin(theta_old)) * delta_t
            Ynew = Yold + (Vx * math.sin(theta_old) + Vy * math.cos(theta_old)) * delta_t
            theta_new = theta_old + (Va * delta_t)

            # Update the global position
            self.robot_position['x'] = Xnew
            self.robot_position['y'] = Ynew
            self.robot_position['angle'] = theta_new

            print(f"Robot Position: {self.robot_position}")
            