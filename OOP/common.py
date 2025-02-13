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


wheel_pos = [0, 0, 0, 0]

def get_wheel_dict():
    return {
        "FL": wheel_pos[0],
        "BL": wheel_pos[1],
        "FR": wheel_pos[2],
        "BR": wheel_pos[3]
    }