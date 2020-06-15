'''
Created on April 19, 2020

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
num_channel = 18
plength = numpy.zeros(num_channel)

# customize pulse length
plength[0] = 20
plength[2] = 0

# sensor address
sen_address = 23
pspac = 200 # one duty cycle (us)
iter = 1    # number of iteration

nmrObj.igbtPulse(plength, pspac, iter, sen_address)