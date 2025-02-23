"""
This module is responsible for plotting the path of the robot as it moves.

Global Variables included:
None

Usage:
This module can be imported as a module for the robot control system of PAUL.

Example for those who aren't experts at python:
from RobotPathPlotter import RobotPathPlotter

Classes:
RobotPathPlotter is the class name but is a subclass of the Position class.

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
seaborn
"""

import seaborn as sns
import matplotlib.pyplot as plt
import time 
import threading
from Velocity import Velocity
from PrintQueue import PrintQueue
from rwSerial import rwSerial
import serial 

class RobotPathPlotter:
    def __init__(self):
        # This will store the positions in a list
        self.x_positions = []
        self.y_positions = []
        self.timestamps = []

    def update_position(self, x, y):
        #This just appends the current x and y positions to the list so new positions are being added
        self.x_positions.append(x)
        self.y_positions.append(y)
        self.timestamps.append(time.time())

    def plot_path(self):
         # Create a new figure with specified size
        plt.figure(figsize=(10, 8))
        
        # Plot the continuous path of the robot
        plt.plot(self.x_positions, self.y_positions, 'b-', label='Robot Path')
        
        # Plot dots for the robot's position every second
        start_time = self.timestamps[0]
        for i, t in enumerate(self.timestamps):
            if i == 0 or t - start_time >= 1.0:
                plt.plot(self.x_positions[i], self.y_positions[i], 'ro', markersize=5)
                start_time = t

        # Set plot title and labels
        plt.title('Robot Path')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')  # Ensure the aspect ratio is equal
        plt.show()  # Display the plot

    def main():
        print_queue = PrintQueue()
        serial_port = serial.Serial("/dev/ttyAMA0", 250000, timeout=1)
        rw_serial = rwSerial(serial_port, print_queue)
        velocity = Velocity(print_queue, rw_serial)
        plotter = RobotPathPlotter()

    def update_plot():
        # Continuously update the plotter with new position data
        while True:
            position = velocity.robot_position
            plotter.update_position(position['x'], position['y'])
            time.sleep(0.1)  # Update plot data every 0.1 seconds

    # Start a separate thread for updating the plot data
    update_thread = threading.Thread(target=update_plot, daemon=True)
    update_thread.start()

    try:
        # Move forward for 5 seconds
        velocity.set_target_velocity(0.1, 0.0, 0.0)
        time.sleep(5)
        # Move right for 5 seconds
        velocity.set_target_velocity(0.0, 0.1, 0.0)
        time.sleep(5)
        # Move backward for 5 seconds
        velocity.set_target_velocity(-0.1, 0.0, 0.0)
        time.sleep(5)
        # Move left for 5 seconds
        velocity.set_target_velocity(0.0, -0.1, 0.0)
        time.sleep(5)
        # Rotate clockwise for 5 seconds
        velocity.set_target_velocity(0.0, 0.0, 0.5)
        time.sleep(5)
    except KeyboardInterrupt:
        # Allow the user to stop the program with Ctrl+C
        pass
    finally:
        # Ensure the serial port is closed when the program exits
        serial_port.close()

    # Generate and display the plot after the movement is complete
    plotter.plot_path()

if __name__ == "__main__":
    main()
