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
    def __init__(self):
        self.queue = queue.Queue()

    def add_message(self, message):
        """Add a message to the queue."""
        self.queue.put(message)

    def print_messages(self):
        """Continuously print messages from the queue."""
        while True:
            try:
                message = self.queue.get(timeout=0.02)
                sys.stdout.write(message + '\n')
                sys.stdout.flush()
            except Empty:
                continue