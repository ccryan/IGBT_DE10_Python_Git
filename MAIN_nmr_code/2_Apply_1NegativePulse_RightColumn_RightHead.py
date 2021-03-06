"""
The script apply 1 negative pulse with tunable duration to one comun in tre right head
The pulses applied to the most right (seeing from negative z(lab frame) direction) column
These are magnets 21 24 and 27
After that it reads back Hall sensors and save them to file
25 readings are performed from each sensor 
"""

import os
import matplotlib.pyplot as plt
import scipy.io as sio
import numpy as np
import datetime
import time
import shutil

from nmr_std_function.nmr_class import tunable_nmr_system_2018
from nmr_std_function.data_parser import parse_csv_returnZreading
from nmr_std_function.kalman_filter_for_fpga import Kalman_Filter
from nmr_std_function.FeedbackController_revision002 import *

# variables
data_folder = "/root/HallSensorData"
en_remote_dbg = 0
nmrObj = tunable_nmr_system_2018(data_folder, en_remote_dbg)

st = datetime.datetime.fromtimestamp(time.time()).strftime('%Y_%m_%d %H_%M_%S')

try:
    os.remove(data_folder + '/Current_Reading.csv') # delete current_reading.csv every time
except:
    print("file does not exist")

# --------------------- Pin address assignement --------------------------------
num_channel = 36
pspac = 200             # maximum pulse length 
pulse_reptition = 1     # No of pulses in one sequence
plength = np.zeros(num_channel)

#plength_forward = numpy.zeros(num_channel)
#plength_backward = numpy.zeros(num_channel)
n_reading = 25  # number of reading 
#sen_address = [28, 29, 31, 33, 32 ,30, 34, 35, 36 ]
sen_address = [19, 20, 21, 22, 23 ,24, 25, 26, 27]
forward_direction_addr = [20, 17, 21, 24, 28, 25, 29, 32, 33] #decrease HS reading
reverse_direction_addr = [22, 19, 23, 26, 30, 27, 31, 34, 35] #increase HS reading

enable_message = False
if enable_message == False:
    print("\n") 
    print("\t Message Disabled")
    print("\n") 
# ------------------------------------------------------------------------------


# -------------------- Controller initialization -------------------------------    
loop_iteration_total = 1


magnets_num = 9
magnets = []
controllers = []

U=np.zeros(magnets_num)

#bh_factor = 0.05#0.5 # Anjana, consider optimizing this variable during autotuning...                                      #.7

Kp = .05                                       # 0.1 .05 0.6 .10 proportional gain - needs tuning
Ki = .15                                      #.06 .1 0.2 0.1875 0.02 integral gain - needs tuning
Kd = 0                                          # differential gain 
beta = 1                                     # .7  .6 setpoint weighting - needs tuning

min_pulse_length, max_pulse_length = -15,15.     # pulse length limits in mico seconds 

# chkbrd = [-2000,2000,-2000,2000,-2000,2000,-2000,2000,-2000]
# setpoints = chkbrd #initialization


setpoints = np.zeros(magnets_num)                # target magnetization in Gauss
value=-1500
setpoints[0] = value#0 #100#
setpoints[1] = value#200 #100#
setpoints[2] = value#0 #100#
setpoints[3] = value#200 #100#
setpoints[4] = value#200#-2000#
setpoints[5] = value#200 #100#                     
setpoints[6] = value#0 #100#
setpoints[7] = value#200 #100#
setpoints[8] = value#0 #100#

bh_factor = np.zeros(magnets_num)                # target magnetization in Gauss
f=1
bh_factor[0] = f  
bh_factor[1] = f 
bh_factor[2] = f
bh_factor[3] = f
bh_factor[4] = f
bh_factor[5] = f                   
bh_factor[6] = f 
bh_factor[7] = f
bh_factor[8] = f 

pulse_on = {}
pulse_on[0] = False
pulse_on[1] = False
pulse_on[2] = True
pulse_on[3] = False
pulse_on[4] = False
pulse_on[5] = True
pulse_on[6] = False
pulse_on[7] = False
pulse_on[8] = True

for n in range(0,magnets_num):
    Kp_weighted = Kp *bh_factor[n]                                     
    Ki_weighted = Ki *bh_factor[n]    
    controllers.append(FeedbackController(Kp_weighted, Ki_weighted, Kd, 
                                        beta, 
                                        setpoint=setpoints[n], 
                                        output_limits=(min_pulse_length,max_pulse_length), 
                                        setpoint_weighting=False,
                                        proportional_on_input=False,
                                        output_enabled=pulse_on[n]))

# ------------------------------------------------------------------------------

for t in range(0, loop_iteration_total):
    plength = np.zeros(num_channel)

    print("Start of iteration {}".format(t))
    
    for n in range(0,magnets_num):     
        # ---------------------------
        #u = controllers[n].update(ZReading_Average)             # compute new ouput. Save value in array U 
        #u = u* (-1)                                             # determine by the pin assignment to the channels
        u = 0
        if pulse_on[n]==True:
            u = -25

        u = u* (-1) 

        if u >= 0:
            plength[forward_direction_addr[n]] = abs(np.int(u))                         # actuate magnets

        if u < 0:
            plength[reverse_direction_addr[n]] = abs(np.int(u))
            
    #shutil.copy(data_folder + '/Current_Reading.csv', data_folder + '/SensorReading_' + st + '_Iter_{}.csv'.format(t))
    #os.remove(data_folder + '/Current_Reading.csv') # delete current_reading.csv every time
    print("\t plength : {}".format(plength))
    nmrObj.igbtPulse(plength, pspac, pulse_reptition)

# read hall sensor value in Tesla
nmrObj.igbtSenReadingMulti(sen_address, n_reading, enable_message)
zReading = parse_csv_returnZreading(data_folder, 'Current_Reading.csv')  # in Gauss
    
for n in range(0,magnets_num):        
    # read hall sensor value in Gauss
    #nmrObj.igbtSenReading(sen_address[n], n_reading)

    zReading_Sensor = zReading[n_reading* n: n_reading*(n+1)]
       
    # kalman filter
    ZReading_Average = Kalman_Filter(n_reading, zReading_Sensor)
    y = ZReading_Average  
    print("\n")   
    print("\tCurrent hall reading is : {}".format(y))
    print("\tSensor Address is : {}".format(sen_address[n]))
    print("\n")      

