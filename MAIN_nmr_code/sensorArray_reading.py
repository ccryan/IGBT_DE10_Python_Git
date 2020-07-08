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

# IGBT Channel setting
num_channel = 36
plength = numpy.zeros(num_channel)

# customize pulse length
#plength[0] = 0
#plength[0] = 50

# sensor address
sen_address = 2
pspac = 200
enable_message = True

# one duty cycle (us)
iter = 1    # number of iteration

# number of readings
n_reading = 5

try:
    os.remove(data_folder + '/Current_Reading.csv') # delete current_reading.csv every time
except:
    print("file does not exist")
       
# read hall sensor value in Tesla
nmrObj.FOVSenReading(n_reading, enable_message)

'''
#zReading = parse_csv_returnZreading(data_folder, 'Current_Reading.csv')  # in Gauss
#os.remove(data_folder + '/Current_Reading.csv') # delete current_reading.csv every time
time.sleep(1.1) 

# kalman filter
ZReading_Average = Kalman_Filter(n_reading, zReading)
y = ZReading_Average
print("\n")   
print("\tCurrent hall reading is : {}".format(y))
print("\tSensor Address is : {}".format(sen_address))
print("\n")      

# nmrObj.igbtPulse(plength, pspac, iter, sen_address)

try:
    os.remove(data_folder + '/Current_Reading.csv') # delete current_reading.csv every time
except:
    print("file does not exist")

# read hall sensor value in Tesla
# nmrObj.igbtSenReading(sen_address, n_reading)
# zReading = parse_csv_returnZreading(data_folder, 'Current_Reading.csv')  # in Gauss
# os.remove(data_folder + '/Current_Reading.csv') # delete current_reading.csv every time

# kalman filter
ZReading_Average = Kalman_Filter(n_reading, zReading)
y = ZReading_Average
print("\n")   
print("\tCurrent hall reading is : {}".format(y))
print("\tSensor Address is : {}".format(sen_address))
print("\n") 
'''     