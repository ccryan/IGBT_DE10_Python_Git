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
from nmr_std_function.kalman_filter_for_fpga import Kalman_Filter
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
n_reading = 20    # number of reading 
sen_address = [19, 20, 21, 22, 23 , 24, 25, 26, 27 ]
forward_direction_addr = [0, 4, 24, 8, 12, 9, 13, 16, 20]
reverse_direction_addr = [2, 6, 26, 10, 14, 11, 15, 18, 22]
enable_message = False
if enable_message == False:
    print("\n") 
    print("\t Message Disabled")
    print("\n") 
# ------------------------------------------------------------------------------


# -------------------- Controller initialization -------------------------------    
loop_iteration_total = 15


magnets_num = 9
magnets = []
controllers = []

U=np.zeros(magnets_num)

Kp = .05                                      # 0.1 .05 0.6 .10 proportional gain - needs tuning
Ki = 0.05                                     #.06 .1 0.2 0.1875 0.02 integral gain - needs tuning
Kd = 0                                       # differential gain 
beta = .5                                    # .7  .6 setpoint weighting - needs tuning

min_pulse_length, max_pulse_length = -50,50.     # pulse length limits in mico seconds 

setpoints = np.zeros(magnets_num)                # target magnetization in mTesla
setpoints[0] = -300
setpoints[1] = 1500
setpoints[2] = 1500
setpoints[3] = 1500
setpoints[4] = 1500
setpoints[5] = 1500                        
setpoints[6] = 1500
setpoints[7] = 1500
setpoints[8] = 1500
"""
pulse_on = {}
pulse_on[0] = False
pulse_on[1] = True
pulse_on[2] = True
pulse_on[3] = True
pulse_on[4] = True
pulse_on[5] = True
pulse_on[6] = True
pulse_on[7] = True
pulse_on[8] = True
"""
#here we define which magnet are controlled with the feedback loop and which are not
pulse_on = {}
pulse_on[0] = True
pulse_on[1] = True
pulse_on[2] = True
pulse_on[3] = True
pulse_on[4] = True
pulse_on[5] = True
pulse_on[6] = True
pulse_on[7] = True
pulse_on[8] = True


for n in range(0,magnets_num):
        
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
    # read hall sensor value in Tesla
    nmrObj.igbtSenReadingMulti(sen_address, n_reading, enable_message)
    print("Start of iteration {}".format(t))
    for n in range(0,magnets_num):        
        # read hall sensor value in Tesla
        nmrObj.igbtSenReading(sen_address[n], n_reading)
        
        zReading = parse_csv_returnZreading(data_folder, 'Current_Reading_{}.csv'.format(sen_address[n]))  # in Gauss
        time.sleep(1.1) 
        
        os.remove(data_folder + '/Current_Reading_{}.csv'.format(sen_address[n])) # delete current_reading.csv every time
        
        # kalman filter
        ZReading_Average = Kalman_Filter(n_reading, zReading)
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
    
    print("\t plength : {}".format(plength))
    nmrObj.igbtPulseMagnetControl(plength, pspac, pulse_reptition)

# save results as MAT file
experiment_data = {}
st = datetime.datetime.fromtimestamp(time.time()).strftime('%Y_%m_%d %H_%M_%S')
for n in range(0,magnets_num):
  experiment_data.update({str(n):{
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
