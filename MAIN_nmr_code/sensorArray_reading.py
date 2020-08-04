'''
Created on June 30, 2020

@author: Cheng Chen
'''

#!/usr/bin/python

import os
import time
import numpy

from nmr_std_function.data_parser import parse_simple_info
from nmr_std_function.nmr_functions import compute_iterate
from nmr_std_function.nmr_class import tunable_nmr_system_2018
from nmr_std_function.data_parser import parse_csv_float2col

from nmr_std_function.data_parser import parse_csv_returnZreading
from nmr_std_function.kalman_filter_for_fpga import Kalman_Filter

import matplotlib.pyplot as plt
from scipy import signal
import pydevd

# variables
data_folder = "/root/HallSensorData"
en_fig = 1
en_remote_dbg = 0

# system setup
nmrObj = tunable_nmr_system_2018(data_folder, en_remote_dbg)

enable_message = True

# one duty cycle (us)
iter = 1    # number of iteration

# number of readings
n_reading = 25

try:
    os.remove(data_folder + '/Current_Reading.csv') # delete current_reading.csv every time
except:
    print("file does not exist")
       
# read hall sensor value in Tesla
nmrObj.FOVSenReading(n_reading, enable_message)

