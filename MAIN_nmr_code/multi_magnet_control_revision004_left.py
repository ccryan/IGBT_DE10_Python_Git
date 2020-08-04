'''
Created on July 10, 2020
Updated on July 21, 2020
Validated: July 21, 2020

Author: Roland Probst
'''

import os
import matplotlib.pyplot as plt
import scipy.io as sio
import numpy as np
import datetime
import time
import socket

from nmr_std_function.nmr_class import tunable_nmr_system_2018
from nmr_std_function.data_parser import parse_csv_returnZreading
from nmr_std_function.kalman_filter import Kalman_Filter
#from nmr_std_function.feedback_controller_class import *
from nmr_std_function.feedback_controller_class_004_AH import *


loop_iteration_total = 20         # control loop iterations
n_reading = 1                     # hall sensor readings for Kalman Filter

save_data = True
plot_results = True
logging_file_name = 'Feedback_Control_Results_' 

# variables
data_folder = "/root/HallSensorData"
en_remote_dbg = 0
nmrObj = tunable_nmr_system_2018(data_folder, en_remote_dbg)

try:
    os.remove(data_folder + '/Current_Reading.csv') # delete current_reading.csv every time
except:
    print("file does not exist")


# --------------------- Pin address assignement --------------------------------
num_channel = 36
pspac = 200                                     # maximum pulse length 
pulse_reptition = 1                             # number of pulses in one sequence
plength = np.zeros(num_channel)

sen_address =            [1, 2, 3, 4, 5 , 6 , 7 , 8 , 9]
reverse_direction_addr = [0, 4, 1, 5, 8 , 12, 11, 13, 16] #decrease HS reading
forward_direction_addr = [2, 6, 3, 7, 10, 14, 9 , 15, 18] #increase HS reading

enable_message = False
if enable_message == False:
    print("\n") 
    print("\t Message Disabled")
    print("\n") 

# -------------------- Controller initialization -------------------------------    
magnets_num = 9
magnets = []
controllers = []

Kp = 0.12                                        # proportional gain - needs tuning
Ki = 0.08                                        # integral gain - needs tuning
Kd = 0                                          # differential gain 
beta = 0.8                                      # setpoint weighting - needs tuning

min_pulse_length, max_pulse_length = -50,50.    # pulse length limits in mico seconds 


setpoints = np.zeros(magnets_num)               # target magnetization in Gauss
value = -230 # Note:avoid multiples of 64                                   
setpoints[0] = value
setpoints[1] = value
setpoints[2] = value
setpoints[3] = value
setpoints[4] = value
setpoints[5] = value                   
setpoints[6] = value
setpoints[7] = value
setpoints[8] = value


pulse_on = {}
value = True
pulse_on[0] = value
pulse_on[1] = value
pulse_on[2] = value
pulse_on[3] = value
pulse_on[4] = value
pulse_on[5] = value
pulse_on[6] = value
pulse_on[7] = value
pulse_on[8] = value


for n in range(0,magnets_num):
    Kp_weight = Kp                             
    Ki_weight = Ki 
    controllers.append(FeedbackController(Kp,Ki,Kd, 
                                        beta, 
                                        setpoint=setpoints[n], 
                                        output_limits=(min_pulse_length,max_pulse_length), 
                                        setpoint_weighting=False,
                                        proportional_on_input=False,
                                        output_enabled=pulse_on[n]))
# ------------------------------------------------------------------------------

for t in range(0, loop_iteration_total):
    plength = np.zeros(num_channel)

    print("Start of iteration ",t,"/",loop_iteration_total)

    # read all hall sensor values in Gauss
    if t > 10:
        n_reading = 1 # increase readings for higher precision at setpoint
    nmrObj.igbtSenReadingMulti(sen_address, n_reading, enable_message)
    zReading = parse_csv_returnZreading(data_folder, 'Current_Reading.csv') 
    
    for n in range(0,magnets_num):        
        
        # get hall sensor value for magnet n             
        zReading_Sensor = zReading[n_reading* n: n_reading*(n+1)]    
       
        # kalman filter for hall sensor noise reduction
        zReading_Sensor_Average = Kalman_Filter(n_reading, zReading_Sensor)
        
        # compute new output
        u = controllers[n].update(zReading_Sensor_Average)                            
        
        # assemble actuation vector
        if u >= 0:
            plength[forward_direction_addr[n]] = abs(np.int(u))      

        if u < 0:
            plength[reverse_direction_addr[n]] = abs(np.int(u))

        print("\tHall Sensor [",sen_address[n],"]","=",zReading_Sensor_Average,"Gauss", "--> Pulse:",round(u,2),"us")

    nmrObj.igbtPulseMagnetControl(plength, pspac, pulse_reptition) # actuate magnets
    print("\t Actuation : {}".format(plength))

# save results as MAT file
if save_data == True:
    experiment_data = {}
    st = datetime.datetime.fromtimestamp(time.time()).strftime('%Y_%m_%d %H_%M_%S')
    for n in range(0,magnets_num):
      experiment_data.update({'magnet_'+str(n):{
          'control_tunings': controllers[n].tunings,
          'beta': controllers[n].beta,
          'setpoint_weighting': controllers[n].setpoint_weighting,
          'proportional_on_input': controllers[n].proportional_on_input,
          'Iteration':controllers[n].time_total,
          'control_pulse':controllers[n].output_data,
          'magnetization':controllers[n].input_data, 
          'error':controllers[n].error_data,
          'setpoint':controllers[n].setpoint,
          'integral':controllers[n].integral_data
          }})
    
    sio.savemat(logging_file_name+st+'.mat', experiment_data) 

# plot results
if plot_results == True:
    t = np.linspace(0,controllers[0].time_total,controllers[0].time_total)
    
    rows, columns = 3,3
    fig,a = plt.subplots(rows,columns)
    fig,b = plt.subplots(rows,columns)
    
    k = 0
    for n in range(0,rows): # rows
        for m in range(0,columns): # columns
          a[n][m].set_title('Magnet '+ str(sen_address[k]))
          #a[n][m].set_xlabel('Iterations')
          a[n][m].set_ylabel('Magnetization (Gauss)')
          a[n][m].plot(t,np.ones(controllers[k].time_total)*setpoints[k],'r-', label='setpoint')  
          a[n][m].plot(t,controllers[k].input_data,'b-', label='magnetization')  
          #a[n][m].plot(t,controllers[k].error_data,'y-', label='error')
          a[n][m].grid(True)
          if k == 0:
              a[n][m].legend(loc='best')
          
          b[n][m].set_title('Magnet '+ str(sen_address[k]))
          #b[n][m].set_xlabel('Iterations')
          b[n][m].set_ylabel('Pulse (us)')
          b[n][m].plot(t,np.ones(controllers[k].time_total)*max_pulse_length,'r--', label='max_pulse') 
          b[n][m].plot(t,np.ones(controllers[k].time_total)*min_pulse_length,'b--', label='min_pulse') 
          b[n][m].plot(t,controllers[k].output_data,'g*-', label='pulse')
          b[n][m].grid(True)
          if k == 0:
              b[n][m].legend(loc='best')
        
          k += 1
    plt.show()
    
    #a.imsave(data_folder + '/magnetization ' + st, format = 'png')
    #b.imsave(data_folder + '/actuation ' + st, format = 'png')
    
    
    
