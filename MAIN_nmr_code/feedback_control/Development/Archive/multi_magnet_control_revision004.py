"""
Revision 003
- updated procedure for saving results to matlab file. Now, data of all magnets saved into one file.
- hall sensor and current driver pin assignment (address) mapping 
"""

import os
import matplotlib.pyplot as plt
import scipy.io as sio
import numpy as np
import datetime
import time

from nmr_std_function.nmr_class import tunable_nmr_system_2018
from nmr_std_function.data_parser import parse_csv_returnZreading
from kalman_filter import Kalman_Filter
from nmr_std_function.FeedbackController_revision002 import *

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
pspac = 200             # maximum pulse length 
pulse_reptition = 1     # No of pulses in one sequence
plength = np.zeros(num_channel)

#plength_forward = numpy.zeros(num_channel)
#plength_backward = numpy.zeros(num_channel)
n_reading = 150    # number of reading 
sen_address = [28, 29, 31, 33, 32 ,30, 34, 35, 36 ]
#sen_address = [19, 20, 31, 33, 32 ,30, 34, 35, 36 ]
forward_direction_addr = [9, 13, 16, 0, 4, 1, 5, 8, 12]
reverse_direction_addr = [11, 15, 18, 2, 6, 3, 7, 10, 14]
enable_message = False
if enable_message == False:
    print("\n") 
    print("\t Message Disabled")
    print("\n") 
# ------------------------------------------------------------------------------


# -------------------- Controller initialization -------------------------------    
loop_iteration_total = 10


magnets_num = 9
magnets = []
controllers = []

U=np.zeros(magnets_num)

#bh_factor = 0.05#0.5 # Anjana, consider optimizing this variable during autotuning...                                      #.7

Kp = .28                                       # 0.1 .05 0.6 .10 proportional gain - needs tuning
Ki = 0.5                                      #.06 .1 0.2 0.1875 0.02 integral gain - needs tuning
Kd = 0                                          # differential gain 
beta = 0.8                                     # .7  .6 setpoint weighting - needs tuning

min_pulse_length, max_pulse_length = -10,10.     # pulse length limits in mico seconds 

"""
chkbrd = [-2000,2000,-2000,2000,-2000,2000,-2000,2000,-2000]
setpoints = chkbrd #initialization
bh_factor = [1,1,1,1,1,1,1,1,1]
"""
sp = [1000,0,1000,0,1000,0,1000,0,1000]
pulse_length = [5,50,10,50,25,50,30,50,50]


setpoints = np.zeros(magnets_num)                # target magnetization in Gauss
bh_factor = np.zeros(magnets_num)                # target magnetization in Gauss
pulse_on = {}

for n in range(0,length(sp)):

    min_pulse_length, max_pulse_length = -pulse_length(n),pulse_length(n)
    
    setpoints[0] = sp(n)#-1000 #100#
    setpoints[1] = 100#200 #100#
    setpoints[2] = 100#0 #100#
    setpoints[3] = 100#200 #100#
    setpoints[4] = 100#200#-2000#
    setpoints[5] = 100#200 #100#                     
    setpoints[6] = 100#0 #100#
    setpoints[7] = 100#200 #100#
    setpoints[8] = 100#0 #100#
    
    if sp(n) == 0:
        bh_factor = [.05,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3]
        
        
        pulse_on[0] = True
        pulse_on[1] = True
        pulse_on[2] = True
        pulse_on[3] = True
        pulse_on[4] = True
        pulse_on[5] = True
        pulse_on[6] = True
        pulse_on[7] = True
        pulse_on[8] = True
        
    elif sp(n) == 1000:
        bh_factor = [1,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3]
        
        
        pulse_on[0] = True
        pulse_on[1] = False
        pulse_on[2] = False
        pulse_on[3] = False
        pulse_on[4] = False
        pulse_on[5] = False
        pulse_on[6] = False
        pulse_on[7] = False
        pulse_on[8] = False
    

    
    
    pulse_on[0] = True
    pulse_on[1] = False
    pulse_on[2] = False
    pulse_on[3] = False
    pulse_on[4] = False
    pulse_on[5] = False
    pulse_on[6] = False
    pulse_on[7] = False
    pulse_on[8] = False
    
    
    
    
    for n in range(0,magnets_num):
        Kp = Kp*bh_factor[n]                                     
        Ki = Ki*bh_factor[n]    
        controllers.append(FeedbackController(Kp,Ki, Kd, 
                                            beta, 
                                            setpoint=setpoints[n], 
                                            output_limits=(min_pulse_length,max_pulse_length), 
                                            setpoint_weighting=True,
                                            proportional_on_input=False,
                                            output_enabled=pulse_on[n]))
    
    # ------------------------------------------------------------------------------
    
    for t in range(0, loop_iteration_total):
        plength = np.zeros(num_channel)
    
        print("Start of iteration {}".format(t))
        # read hall sensor value in Tesla
        nmrObj.igbtSenReadingMulti(sen_address, n_reading, enable_message)
        zReading = parse_csv_returnZreading(data_folder, 'Current_Reading.csv')  # in Gauss
        
        for n in range(0,magnets_num):        
            # read hall sensor value in Tesla
            # nmrObj.igbtSenReading(sen_address[n], n_reading)
    
            zReading_Sensor = zReading[n_reading* n: n_reading*(n+1)]
           
            # kalman filter
            ZReading_Average = Kalman_Filter(n_reading, zReading_Sensor)
            y = ZReading_Average  
            print("\n")   
            print("\tCurrent hall reading is : {}".format(y))
            print("\tSensor Address is : {}".format(sen_address[n]))
            print("\n")      
            
            # ---------------------------
            u = controllers[n].update(ZReading_Average)             # compute new ouput. Save value in array U 
            u = u* (-1)                                             # determine by the pin assignment to the channels
            
            if u >= 0:
                plength[forward_direction_addr[n]] = abs(np.int(u))                         # actuate magnets
    
            if u < 0:
                plength[reverse_direction_addr[n]] = abs(np.int(u))
        
        os.remove(data_folder + '/Current_Reading.csv') # delete current_reading.csv every time
        print("\t plength : {}".format(plength))
        nmrObj.igbtPulseMagnetControl(plength, pspac, pulse_reptition)
    
    # save results as MAT file
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
          'setpoint':controllers[n].setpoint
          }})
    
    sio.savemat('Feedback_Control_Results_'+st+'.mat', experiment_data) 
    
    

# plot results
"""
t = np.linspace(0,controllers[0].time_total,controllers[0].time_total)

rows, columns = 3,3
fig,a = plt.subplots(rows,columns)
fig,b = plt.subplots(rows,columns)

k = 0
for n in range(0,rows): # rows
    for m in range(0,columns): # columns
      a[n][m].set_title('Magnet '+ str(sen_address[k]))
      #a[n][m].set_xlabel('Iterations')
      a[n][m].set_ylabel('Magnetization (mT)')
      a[n][m].plot(t,np.ones(controllers[k].time_total)*setpoints[k],'r-', label='setpoint')  
      a[n][m].plot(t,controllers[k].input_data,'b-', label='magnetization')  
      #a[n][m].plot(t,controllers[k].error_data,'y-', label='error')
      a[n][m].grid(True)
      a[n][m].legend(loc='best')
      
      b[n][m].set_title('Magnet '+ str(sen_address[k]))
      #b[n][m].set_xlabel('Iterations')
      b[n][m].set_ylabel('Pulse (us)')
      b[n][m].plot(t,np.ones(controllers[k].time_total)*max_pulse_length,'r--', label='max_pulse') 
      b[n][m].plot(t,np.ones(controllers[k].time_total)*min_pulse_length,'b--', label='min_pulse') 
      b[n][m].plot(t,controllers[k].output_data,'g*-', label='pulse')
      b[n][m].grid(True)
      b[n][m].legend(loc='best')
      k += 1
plt.show()
"""