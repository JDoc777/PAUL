#!/user/bin/env python3
import serial #name of pyserial library
import time

#from here down to line 9 similar to void setup
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0) #some people use a baudrate of 9600
time.sleep(3) #When you open the serial communication from this side, the arduino side will be restarted. Give enough time for it to be ready
ser.reset_input_buffer() #when the data arrives it goes to a buffer, and we read from the buffer in case we get sent info before the buffer
print("Serial OK")

try:
    #create infinite loop to read the string
    while True: #Infinite loop to read serial communication
        time.sleep(1); 
        ser.write("Button counter : \n".encode('utf-8'))
        while ser.in_waiting <= 0:
            time.sleep(0.01)
        response = ser.readline().decode('utf-8')
        
        print(response)
        
        
except KeyboardInterrupt: #When the user presses ctrl + C it will end 
    print("Closing serial communication....")
    ser.close()

        
    
