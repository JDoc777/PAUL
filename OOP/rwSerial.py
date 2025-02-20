"""
This script demonstrates how to read and parse serial data from a microcontroller

Global Variables included:
None

Usage:
This module can be imported as a module for the robot control system of PAUL.

Example for those who aren't experts at python:
from rwSerial import rwSerial

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

#from common import wheel_pos, get_wheel_dict
baud_rate = 250000          
ser = "/dev/ttyAMA0"
serial_port = serial.Serial(ser, baud_rate, timeout=1)

class rwSerial:    
    def __init__(self, serial_port, print_queue):
        self.serial_port = serial_port
        self.print_queue = print_queue
        self.encoderPos_FL = 0
        self.encoderPos_BL = 0
        self.encoderPos_FR = 0
        self.encoderPos_BR = 0
        #self.wheel_pos = [self.encoderPos_FL, self.encoderPos_BL, self.encoderPos_FR, self.encoderPos_BR]
    
    
                    
        
#     def add_to_queue(self, message):
#         slef.queue.put(message)
#         
#     
#     def print_queue_data(self):
#         
#         while True:
#             try:
#                 message = print_queue.get(timeout=0.02)
#                 sys.stdout.write(message + '\n')
#                 sys.stdout.flush()
#             except Empty:
#                 continue

        
    def parse_serial(self):
        
        while True:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()  # Decode with error handling
                    
                    if line:
                        self.print_queue.add_message(f"Raw data: {line}")  # Print the raw incoming data
                        
                        try:
                            # Parse JSON data
                            data = json.loads(line)
                            
                            # Extract various sensor data (adjust to match your actual structure)
                            distance_f = data['distance']['F']
                            distance_b = data['distance']['B']
                            distance_l = data['distance']['L']
                            distance_r = data['distance']['R']
                    
                            gyro_x = data['gyro']['x']
                            gyro_y = data['gyro']['y']
                            gyro_z = data['gyro']['z']
                    
                            accel_x = data['accel']['x']
                            accel_y = data['accel']['y']
                            accel_z = data['accel']['z']
                            
                            encoderPos_FL = data['enco']['1']['position'] 
                            
                            encoderPos_BL = data['enco']['2']['position'] 
                           
                            encoderPos_FR = data['enco']['3']['position']
                            
                            encoderPos_BR = data['enco']['4']['position']
                            
                                                       
                         
                                
                            rotation_1 = data['enco']['1']['rotation']
                            rotation_2 = data['enco']['2']['rotation']
                            rotation_3 = data['enco']['3']['rotation']
                            rotation_4 = data['enco']['4']['rotation']

                            temperature = data['environment']['temperature']
                            humidity = data['environment']['humidity']
                            #print(wheel_dict)
                            # Print parsed data
                            self.print_queue.add_message(f"Distances - L: {distance_l}, R: {distance_r}, B: {distance_b}, F: {distance_f}")
                            self.print_queue.add_message(f"Gyro - X: {gyro_x}, Y: {gyro_y}, Z: {gyro_z}")
                            self.print_queue.add_message(f"Accel - X: {accel_x}, Y: {accel_y}, Z: {accel_z}")
                            self.print_queue.add_message(f"Encoder
                                                          1 - Position: {encoderPos_FL}, Rotation: {rotation_1}")
                            self.print_queue.add_message(f"Encoder 2 - Position: {encoderPos_FR}, Rotation: {rotation_2}")
                            self.print_queue.add_message(f"Encoder 3 - Position: {encoderPos_BL}, Rotation: {rotation_3}")
                            self.print_queue.add_message(f"Encoder 4 - Position: {encoderPos_BR}, Rotation: {rotation_4}")
                            self.print_queue.add_message(f"Environment - Temperature: {temperature}°C, Humidity: {humidity}%")
                            
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
                
        return encoderPos_FL
         
def main():
    

    print_queue = PrintQueue.PrintQueue()
    obj = rwSerial(serial_port,print_queue)
    velo = Velocity.Velocity(print_queue,obj)
    encoder = Encoder.Encoder(print_queue,obj)
    position = Position.Position(print_queue,obj)
    
#     FL, BL, FR, BR = encoder.choose_wheel()
#     print("{FL}")
    
#     for i in range(len(encoder.choose_wheel())):
#                    
#         print(encoder.choose_wheel()[i])
#         
#     prev_pos = encoder.choose_wheel()[0]
    #obj.parse_serial()
    #obj.print_queue_data()
   
    #encoder_thread = threading.Thread(target=encoder.test_queue, daemon=True)
    #velocity_thread = threading.Thread(target=velo.test_print, daemon =True)

    #added robot_position
    robot_position = threading.Thread(target=velo.update_robot_position, daemon =True)
    #velocity_thread = threading.Thread(target=velo.calculate_velocity_components, daemon =True)
    #encoder_thread = threading.Thread(target=encoder.choose_wheel, daemon=True)
    #parse_thread = threading.Thread(target=obj.parse_serial,daemon=True)
    print_thread = threading.Thread(target=print_queue.print_messages, daemon=True)
    
    
    robot_position.start()
    #velocity_thread.start()
    #encoder_thread.start() 
    #parse_thread.start()
    print_thread.start()
    
    print_thread.join()
    robot_position.join()
    #encoder_thread.join()
    #velocity_thread.join()

    print_queue.add_message("Starting UART communication...")
    #time.sleep(0.1)
    
    try:
        while True:
            time.sleep(0.01)  # Keep the main thread alive
    except KeyboardInterrupt:
        print_queue.add_message("Stopping communication...")
    finally:
        
        serial_port.close()
 
  
if __name__ == "__main__":
    main()