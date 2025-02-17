"""
This Velocity module is responsible for calculating the velocity of the robot using the encoder data and updating the robot's position based on the wheel velocities.

Global Variables included:
None 

Usage:
This module can be imported as a module for the robot control system of PAUL.

Example for those who aren't experts at python:
from Velocity import Velocity

Classes:
Velocity is the class name but is a subclass of the Position class. 
It is responsible for calculating the velocity of the robot using the encoder data and updating the robot's position based on the wheel velocities.

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
import numpy as np
import Position # When adding libraries make sure to use as to name them something besices the library or module name unless it a class from a pyhthon library that is commonly used

class Velocity(Position.Position):
    
    def __init__(self, print_queue,rwSerial_instance): #fix this
        super().__init__(print_queue, rwSerial_instance)
        self.rwSerial_instance = rwSerial_instance
        self.encoder = Encoder.Encoder(print_queue, self.rwSerial_instance)
        self.print_queue = print_queue
        #self.prev_pos, self.current_encoder_pos, self.delta_t =
        self.robot_position = {'x': 0.0, 'y': 0.0, 'angle': 0.0}
    
    def calculate_velocity(self, wheel):
        
            prev_pos = wheel[0]
            current_encoder_pos = wheel[1]
            delta_t = wheel[2]
                    
            radius = 0.04  # Radius in meters (40 mm)
            pulses_per_revolution = 111.25  # Pulses per revolution (PPR)

        # Calculate the change in encoder position
            delta_n = current_encoder_pos - prev_pos
            
        # Calculate velocity using the formula
            if delta_t > 0:
                velocity = (2 * math.pi * radius * delta_n) / (pulses_per_revolution * delta_t)
                velocity = velocity * 1.8
                #self.print_queue.add_message(f"Velocity: {velocity}")
                return velocity
                
    def wheel_velocities(self):
        FL, BL, FR, BR , dt = self.encoder.choose_wheel()
        FL_velocity = self.calculate_velocity(FL)
        BL_velocity = self.calculate_velocity(BL)
        FR_velocity = self.calculate_velocity(FR)
        BR_velocity = self.calculate_velocity(BR)
#         self.print_queue.add_message(f"""
#             FL = {FL_velocity}
#             BL = {BL_velocity}
#             FR = {FR_velocity}
#             BR = {BR_velocity}
# """)
        return FL_velocity, BL_velocity, FR_velocity, BR_velocity
            
    def calculate_velocity_components(self):
        
        #removed while true ???????
        Vfl, Vrl, Vfr, Vrr = self.wheel_velocities()
        # Wheel radius
        R = 0.04
        
        # half the distance between the wheels
        lx = 0.12
        ly = 0.0725
        lx_ly = lx + ly
        
        # Define the transformation matrix
        T = (R / 4) * np.array([
            [1,  1,  1,  1],
            [-1, 1,  1, -1],
            [-1/lx_ly, -1/lx_ly, 1/lx_ly, 1/lx_ly]
        ])
        
        # Define wheel speeds as a column vector
        omega = np.array([[Vfl], [Vrl], [Vfr], [Vrr]])  # Replace with actual values
        
        # Compute the resulting velocity vector
        velocity = np.dot(T, omega)
        
        Vx = velocity[0]
        Vy = velocity[1]
        Va = velocity[2]

#         #(Vfr - Vfl + Vrr - Vrl)
#         Vx = (R / 4) * (Vfr + Vfl + Vrr + Vrl)
#         Vy = (R / 4) * (Vfr + Vfl + Vrr - Vrl)
#         Va = (R / (4 * (lx + ly) ) ) * (Vfr - Vfl - Vrr + Vrl)
        
        self.print_queue.add_message(f"""
                    VX = {Vx}
                    VY = {Vy}
                    VA = {Va}
        """)

        return Vx, Vy, Va #added return
    
    #added this function
    def update_robot_position(self):
        while True:
            Vx, Vy, Va = self.calculate_velocity_components()
            delta_t = self.encoder.choose_wheel()[4]
            
            # Current position
            Xold = self.robot_position['x']
            Yold = self.robot_position['y']
            theta_old = self.robot_position['angle']
            delta_t = 1

            # Calculate new position
            Xnew = Xold + (Vx * math.cos(theta_old) - Vy * math.sin(theta_old)) * delta_t
            Ynew = Yold + (Vx * math.sin(theta_old) + Vy * math.cos(theta_old)) * delta_t
            theta_new = theta_old + (Va * delta_t)
            #theta_new = (theta_new + math.pi) % (2 * math.pi) - math.pi

            # Update the global position
            self.robot_position['x'] = Xnew
            self.robot_position['y'] = Ynew
            self.robot_position['angle'] = theta_new

            print(f"Robot Position: {self.robot_position}")
            #return robot_position
    
    def test_print(self):
        print(f"Previous Position: {1}")
        print(f"Current Position: {2}")
        print(f"Delta T: {3}")
