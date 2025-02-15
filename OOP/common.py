"""
Wheel Position Modules

This module will provide the details of the wheel positions for the robot control system of PAUL.
It includes global variables for the wheel positions and has a function to retrieve those wheel position as a dictionary for OOP purposes.

Global Varibles included:
    wheel_pos (list): A list of four integers representing the positions of the wheels.
    The order of the wheels are [Front Left, Back Left, Front Right, Back Right].

    Functions:
    get_wheel_positions(): returns the position of the wheels and thier values from the list

    Usage:
    This module can be imported as a module for the robot control system of PAUL.

    Example for those who aren't experts at python:
    from wheel_position import wheel_pos, get_wheel_dic

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


wheel_pos = [0, 0, 0, 0]

# get_wheel_dict allows you to
# return a list of the wheel positions.
# FL (Front Left) index 0
# BL (Back Left) index 1
# FR (Front Right) index 2
# BR (Back Right) index 3

def get_wheel_dict():
    return {
        "FL": wheel_pos[0],
        "BL": wheel_pos[1],
        "FR": wheel_pos[2],
        "BR": wheel_pos[3]
    }