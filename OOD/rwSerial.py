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

baud_rate = 250000          
ser = "/dev/ttyAMA0"
serial_port = serial.Serial(ser, baud_rate, timeout=1)


print_queue = queue.Queue()
 

wheel_pos = [0,0,0,0]
class rwSerial:
    encoderPos_FL = 0
    
    encoderPos_BL = 0
                           
    encoderPos_FR = 0
                      
                      
    encoderPos_BR = 0
    
    
    wheel_pos = [encoderPos_FL,encoderPos_BL,encoderPos_FR,encoderPos_BR]

    
                            
   
   
                    
    wheel_dict = {
                        
                        "FL": wheel_pos[0],
                        "BL": wheel_pos[1],
                        "FR": wheel_pos[2],
                        "BR": wheel_pos[3]
                        }
    
    def __init__(self, serial_port, print_queue):
        self.serial_port = serial_port
        self.print_queue = print_queue
        
    def add_to_queue(self, message):
        slef.queue.put(message)
        
    
    def print_queue_data(self):
        
        while True:
            try:
                message = print_queue.get(timeout=0.02)
                sys.stdout.write(message + '\n')
                sys.stdout.flush()
            except Empty:
                continue
    
                
    def parse_serial(self):
        
        while True:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()  # Decode with error handling
                    
                    if line:
                        print_queue.put(f"Raw data: {line}")  # Print the raw incoming data
                        
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
                            
                            wheel_pos[0] = encoderPos_FL #index 0 ; Front Left
                            wheel_pos[1] = encoderPos_BL #index 1 ; Back Left
                            wheel_pos[2] = encoderPos_FR #index 2 ; Front Right
                            wheel_pos[3] = encoderPos_BR #index 3 ; Back Right
                            
                            wheel_dict = {
                                
                                "FL": wheel_pos[0],
                                "BL": wheel_pos[1],
                                "FR": wheel_pos[2],
                                "BR": wheel_pos[3]
                                }

                            rotation_1 = data['enco']['1']['rotation']
                            rotation_2 = data['enco']['2']['rotation']
                            rotation_3 = data['enco']['3']['rotation']
                            rotation_4 = data['enco']['4']['rotation']

                            temperature = data['environment']['temperature']
                            humidity = data['environment']['humidity']
                            print(wheel_dict)
                            # Print parsed data
                            self.print_queue.put(f"Distances - L: {distance_l}, R: {distance_r}, B: {distance_b}, F: {distance_f}")
                            self.print_queue.put(f"Gyro - X: {gyro_x}, Y: {gyro_y}, Z: {gyro_z}")
                            self.print_queue.put(f"Accel - X: {accel_x}, Y: {accel_y}, Z: {accel_z}")
                            self.print_queue.put(f"Encoder 1 - Position: {encoderPos_FL}, Rotation: {rotation_1}")
                            self.print_queue.put(f"Encoder 2 - Position: {encoderPos_FR}, Rotation: {rotation_2}")
                            self.print_queue.put(f"Encoder 3 - Position: {encoderPos_BL}, Rotation: {rotation_3}")
                            self.print_queue.put(f"Encoder 4 - Position: {encoderPos_BR}, Rotation: {rotation_4}")
                            self.print_queue.put(f"Environment - Temperature: {temperature}°C, Humidity: {humidity}%")
                            
                        except json.JSONDecodeError:
                            self.print_queue.put("Error: Received invalid JSON data.")
                        except KeyError as e:
                            self.print_queue.put(f"Error: Missing key in JSON data - {e}")
                    else:
                        self.print_queue.put("No data received.")
                time.sleep(0.01)  # Reduced delay to prevent lag
            except Exception as e:
                self.print_queue.put(f"Error while reading serial data: {e}")
                traceback.print_exc()
                
        
        
        def get_wheel_FL(self):
            return encoderPos_FL
   
            
def main():
    obj = rwSerial(serial_port,print_queue)
    encoder = Encoder.Encoder()
    velo = Velocity.Velocity()
    
    #obj.parse_serial()
    #obj.print_queue_data()
   
    #encoder_thread = threading.Thread(target=encoder.test_queue, daemon=True)
    velocity_thread = threading.Thread(target=velo.test_print(), daemon =True)
    #velocity_thread = threading.Thread(target=velo.calculate_velocity, args=(encoder.get_prev_pos(), encoder.get_current_encoder_pos(), encoder.get_delta_t()), daemon =True)
    encoder_thread = threading.Thread(target=encoder.choose_wheel, args= (rwSerial.wheel_dict,), daemon=True)
    parse_thread = threading.Thread(target=obj.parse_serial, daemon=True)
    print_thread = threading.Thread(target=obj.print_queue_data, daemon=True)
    
    velocity_thread.start()
    encoder_thread.start() 
    parse_thread.start()
    print_thread.start()
    print_queue.put("Starting UART communication...")
    #time.sleep(0.1)
    
    try:
        while True:
            time.sleep(0.01)  # Keep the main thread alive
    except KeyboardInterrupt:
        print_queue.put("Stopping communication...")
    finally:
        
        serial_port.close()
 
  
if __name__ == "__main__":
    main()
