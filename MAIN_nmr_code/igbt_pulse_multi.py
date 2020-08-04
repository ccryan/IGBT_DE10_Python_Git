'''
Created on Jul 10, 2020

@author: Cheng Chen
multi IGBT pulses
'''

#!/usr/bin/python

import os
import time
import numpy
import shutil
import datetime
import numpy as np

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

st = datetime.datetime.fromtimestamp(time.time()).strftime('%Y_%m_%d %H_%M_%S')

# system setup
nmrObj = tunable_nmr_system_2018(data_folder, en_remote_dbg)

# IGBT Channel setting
num_channel = 36
plength = np.zeros(num_channel)

### Sensor Setting
# sensor address
magnets_num = 9
sen_address = [19, 20, 21, 22, 23 ,24, 25, 26, 27]
forward_direction_addr = [20, 17, 21, 24, 28, 25, 29, 32, 33] #decrease HS reading
reverse_direction_addr = [22, 19, 23, 26, 30, 27, 31, 34, 35] #increase HS reading
# number of readings
n_reading = 25
# Show message
enable_message = True


### Pulsing Setting
magnets_num = 9
# pulse duration limit
pspac = 200
# define direction
forward_direction = True
# one duty cycle (us)
pulse_reptition = 1    # number of pulses for one iteration

# number of iteration
loop_iteration_total = 1
rows, cols = (loop_iteration_total, magnets_num) 
plength_iter = [[0 for i in range(cols)] for j in range(rows)] 

# customize pulse length
#         Channel 1,  2,  3,  4,  5,  6,  7,  8,  9
plength_iter  = [[0,  0,  50,  0,  0,  50,  0,  0,  50]]

try:
    os.remove(data_folder + '/Current_Reading.csv') # delete current_reading.csv every time
except:
    print("file does not exist")

nmrObj.igbtSenReadingMulti(sen_address, n_reading, enable_message)
zReading = parse_csv_returnZreading(data_folder, 'Current_Reading.csv')  # in Gauss

for n in range(0,magnets_num):        
    zReading_Sensor = zReading[n_reading* n: n_reading*(n+1)]
   
    # kalman filter
    ZReading_Average = Kalman_Filter(n_reading, zReading_Sensor)
    y = ZReading_Average  
    print("\n")   
    print("\tCurrent hall reading is : {}".format(y))
    print("\tSensor Address is : {}".format(sen_address[n]))
    print("\n")  

for t in range(0, loop_iteration_total):
    print("Start of iteration {}".format(t))
    for n in range(0,magnets_num):  
        plength_tmp = plength_iter[t]
        if forward_direction:
            plength[forward_direction_addr[n]] = plength_tmp[n]
            #nmrObj.igbtPulseMagnetControl(plength_iter[t], pspac, pulse_reptition)
        else:
            plength[reverse_direction_addr[n]] = plength_tmp[n]
            #nmrObj.igbtPulseMagnetControl(plength_iter[t], pspac, pulse_reptition)
    print(plength)            
    time.sleep(1)
    nmrObj.igbtPulseMagnetControl(plength, pspac, pulse_reptition)
    
 
try:
    os.remove(data_folder + '/Current_Reading.csv') # delete current_reading.csv every time
except:
    print("file does not exist")

 
