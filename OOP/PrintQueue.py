"""
Print Queue module 

This module will help provide a thread safe printing when using a queue.
Meaning it will allow for asynchronoyus messsages logging in a multi-threaded environment.

Classes: 
    PrintQueue: This class or file name whatever you want to address it by is a class that 
    manages the queue of messages that are to be printed.
    
Usage: 
    This module can be imported as a module due to OOP principles for the robot control system of PAUL.

Example for those who aren't experts at python:
    from print_Queue import PrintQueue (This is the class name in the file that you cna inherit into other classes)
     
    pq = PrintQueue() (This is the object that you can use to add messages to the queue)
    pq.add_message("Hello World") (This is how you add a message to the queue)
    threading.Thread(target=pq.print_messages, daemon=True).start() (This is how you start the thread to print the messages)
    print_messages, deamon=true).start() (This is how you start the thread to print the messages)

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

class PrintQueue:
    # This class allwos for the queue of messages to be printed.
    def __init__(self): # This is a consturctor in which for OOP principles we use to initialize the object of the class or same file.
        self.queue = queue.Queue()
    
    def add_message(self, message): 
        #Add a message to the queue.
        self.queue.put(message)

    def print_messages(self):
        #Continuously print messages from the queue.
        while True:
            try:
                message = self.queue.get(timeout=0.02)
                sys.stdout.write(message + '\n')
                sys.stdout.flush()
            except Empty:
                continue