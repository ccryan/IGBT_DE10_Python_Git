import os
import matplotlib.pyplot as plt
import scipy.io as sio
import numpy as np
import datetime
import time

from nmr_std_function.nmr_class import tunable_nmr_system_2018
from nmr_std_function.data_parser import parse_csv_returnZreading
from nmr_std_function.kalman_filter import Kalman_Filter
from nmr_std_function.feedback_controller_class import *

# variables
data_folder = "/root/HallSensorData"
en_remote_dbg = 0
nmrObj = tunable_nmr_system_2018(data_folder, en_remote_dbg)

try:
    os.remove(data_folder + '/Current_Reading.csv') # delete current_reading.csv every time
except:
    print("file does not exist")
    
magnets_num = 3
magnets = []
controllers = []

U=np.zeros(magnets_num)

min_pulse_length, max_pulse_length = -50,50.     # pulse length limits in mico seconds 

Kp = 0.05                                        # 0.6 .10 proportional gain - needs tuning
Ki = 0.1875                                    # 0.1875 0.02 integral gain - needs tuning
beta =  0.7                                      # setpoint weighting - needs tuning

Kd = 0

setpoints = [800, 800, 800, 0, 0, 0]                           # target magnetization in mTesla

num_channel = 18
pspac = 200             # maximum pulse length 
pulse_reptition = 1     # No of pulses in one sequence
plength = np.zeros(num_channel)

#plength_forward = numpy.zeros(num_channel)
#plength_backward = numpy.zeros(num_channel)
n_reading = 5    # number of reading 
sen_address = [31, 32 , 33, 29, 30, 33, 34, 35, 36]
forward_direction_addr = [0, 4, 1, 5, 9, 13]
reverse_direction_addr = [2, 6, 3, 7, 11, 15]


for n in range(0,magnets_num):
  controllers.append(FeedbackController(Kp,Ki, 0, 
                                        beta, 
                                        setpoint=setpoints[n], 
                                        output_limits=(min_pulse_length,max_pulse_length), 
                                        setpoint_weighting=False,
                                        proportional_on_input=True,
                                        output_enabled=True))
  
for t in range(0, 25):
    plength = np.zeros(num_channel)
    for n in range(0,magnets_num):
        # read hall sensor value in Tesla
        nmrObj.igbtSenReading(sen_address[n], n_reading)
        zReading = parse_csv_returnZreading(data_folder, 'Current_Reading.csv')  # in Gauss
        time.sleep(1.1) 
        
        os.remove(data_folder + '/Current_Reading.csv') # delete current_reading.csv every time
        
        # kalman filter
        ZReading_Average = Kalman_Filter(n_reading, zReading)
        y = ZReading_Average
        print("\n")   
        print("\tCurrent hall reading is : {}".format(y))
        print("\tSensor Address is : {}".format(sen_address[n]))
        print("\n")      
        
        # ---------------------------
        u = controllers[n].update(ZReading_Average)              # compute new ouput. Save value in array U 
        u = u* (-1)                                             # determine by the pin assignment to the channels
        
        if u >= 0:
            plength[forward_direction_addr[n]] = abs(np.int(u))                         # actuate magnet

        if u < 0:
            plength[reverse_direction_addr[n]] = abs(np.int(u))
    
    print("\t plength : {}".format(plength))
    nmrObj.igbtPulseMagnetControl(plength, pspac, pulse_reptition)

# save results as MAT file
st = datetime.datetime.fromtimestamp(time.time()).strftime('%Y_%m_%d %H_%M_%S')
for n in range(0,magnets_num):
  sio.savemat('Feedback_Control_Results_(Magnet_'+str(n)+')_'+st+'.mat', 
              {'control_tunings': controllers[n].tunings,
               'beta': controllers[n].beta,
               'setpoint_weighting': controllers[n].setpoint_weighting,
               'proportional_on_input': controllers[n].proportional_on_input,
               'Iteration':controllers[n].time_total,
               'Control':controllers[n].output_data,
               'Magnetization':controllers[n].input_data, 
               'Error':controllers[n].error_data})

# plot results
t = np.linspace(0,controllers[0].time_total,controllers[0].time_total)

for n in range(0,magnets_num):
  #plt.subplot(10,3,n+1)
  plt.figure(n,figsize=(10,5))
  plt.xlabel('Iterations')
  plt.ylabel('Magnet '+str(n)+'\n Magnetization (mT)')
  plt.plot(t,controller.input_data,'g-', label='input') 
  plt.plot(t,controller.output_data,'b-', label='output') 
  plt.plot(t,controller.error_data,'r-', label='error')
  plt.legend()

  plt.figure()
  plt.xlabel('Iterations')
  plt.ylabel('Magnet '+str(n)+'\n Control (micro seconds)')
  plt.plot(t,controllers[n].output_data,'g-')
  plt.grid(True)
  
plt.show()
