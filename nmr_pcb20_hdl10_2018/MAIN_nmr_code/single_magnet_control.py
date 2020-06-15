# -*- coding: utf-8 -*-
'''Controller (single magnet).ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1xHsLZKQG4Xu0sqJRk1ItPhfVPcB5G_ht
'''

import matplotlib.pyplot as plt
import scipy.io as sio
import numpy as np
import datetime
import time

import numpy
from nmr_std_function.nmr_class import tunable_nmr_system_2018
from nmr_std_function.data_parser import parse_csv_returnZreading
from nmr_std_function.kalman_filter_for_fpga import Kalman_Filter

# variables
data_folder = "/root/HallSensorData"
en_remote_dbg = 1
nmrObj = tunable_nmr_system_2018(data_folder, en_remote_dbg)

# sensor address
sen_address = 99

num_channel = 18
plength_forward = numpy.zeros(num_channel)
plength_backward = numpy.zeros(num_channel)

current_total = []                                # variables for data logging
magnetization_total = []
error_total = []
time_total = []

I = 0                                             # initialize integral state

Kp = .1                                        # 0.2proportional gain - needs tuning
Ki =  0.015                                       # 0.02 integral gain - needs tuning
b =  .5                                          # setpoint weighting - needs tuning

maxDutyCycle = 20;                                # duty cycle limits
minDutyCycle = -20;

setpoint = 0                                 # target magnetization in mTesla
n_iter = 50                                     # number of iteration for hall reading
    
for t in range(0,25):                             # control algorithm main loop
    r = setpoint                                  # read target magnetization value, e.g. from GUI

    # --------------------------------------
    # read hall sensor value in Tesla
    nmrObj.igbtSenReading(sen_address, n_iter)
    zReading = parse_csv_returnZreading(data_folder, 'Current_Reading.csv')  # in Gauss
    time.sleep(0.1)  
        
        # average
    #ZReading_Average = sum(zReading)/len(zReading)/10      # transfer to mT
    # kalman filter
    ZReading_Average = Kalman_Filter(n_iter, zReading)
    y = ZReading_Average
    print("\n")   
    print("\tCurrent hall reading is : {}".format(y))
    print("\n") 
    
    # --------------------------------------
    #print(y)
    e = r - y 
    #print(e)                                    # calculate setpoint tracking error
    P = Kp*(b*r - y)                              # compute proportional part
    u = P + I                                     # compute output 
    
    if u > maxDutyCycle:                          # anti-windup strategy
        u = maxDutyCycle
        #plength_forward[0] = u
        #plength_forward[0] = 20
        
    elif u < minDutyCycle:
        u = minDutyCycle
        #plength_backward[2] = u
        #plength_backward[2] = 20
    else:
        I = I + Ki*(r-y)  
               
    # update integral state
    
    # --------------------------------------
    # insert magnet actuation code below:
    # customize pulse length
    #plength_forward[0] =u
    #plength_backward[2] = 20
    
    pspac = 200 # one duty cycle (us)
    iter = 1    # number of iteration
    
    if u >= 0:
        plength_forward[0] = abs(np.int(u))                         # actuate magnet
        nmrObj.igbtPulseMagnetControl(plength_forward, pspac, iter)
      # positive_channel(u)                       # positive current channel
    if u < 0:
        plength_backward[2] = abs(np.int(u))
        nmrObj.igbtPulseMagnetControl(plength_backward, pspac, iter)
      # negative_channel(u)                       # negative current channel
    # --------------------------------------

    time_total.append(t)                          # data logging
    current_total.append(u)
    magnetization_total.append(y)
    error_total.append(e)

    time.sleep(0.1)                               # delay loop execution to wait for coil magnetization                               

# save results as MAT file
st = datetime.datetime.fromtimestamp(time.time()).strftime('%Y_%m_%d %H_%M_%S')
sio.savemat('Feedback_Control_Results '+st+'.mat', {'Iteration':time_total,'Current':current_total,'Magnetization':magnetization_total, 'Error':error_total,'control_param':[Kp,Ki,b]})

# plot results with mathplotlib
plt.figure(figsize=(9, 9))

plt.subplot(211)
plt.plot(time_total, current_total, 'b-o')
plt.xlabel('Iteration')
plt.ylabel('Duty Cycle (%)')
plt.grid(True)

plt.subplot(212)
plt.plot(time_total, magnetization_total, 'b-o', error_total, 'r-o')
plt.xlabel('Iteration')
plt.ylabel('Magnetization (mT)')
plt.grid(True)

plt.suptitle('Feedback Control of Single Electropermanent Magnet')
plt.show()
