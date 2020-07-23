'''
Created on July 10, 2020
Updated on July 23, 2020

Notes: 
-code is now automated to run based on an excel spreasheet of values. 
-magnet pulsing and data saveing had been placed inside a method called pulseMagnet

Author: Roland Probst and Anjana Heva
'''
import tkinter as tk
from tkinter import filedialog
import pandas as pd #must install pandas first, or use pipe

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


loop_iteration_total = 50         # control loop iterations
n_reading = 1                     # hall sensor readings for Kalman Filter

save_data = True
plot_results = True
logging_file_name = 'BH_curve_automated_' 

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
sen_address = [28, 29, 31, 33, 32, 30, 34, 35, 36]
reverse_direction_addr = [0, 4, 1, 5, 8, 12, 9, 13, 16]
forward_direction_addr = [2, 6, 3, 7, 10, 14, 11, 15, 18]

enable_message = False
if enable_message == False:
    print("\n") 
    print("\t Message Disabled")
    print("\n") 

# -------------------- Controller initialization -------------------------------    

abs_pl = range(5, 55, 5)
setpoints = []
pulse_on = []
param_dict = dict()

root = tk.Tk()
canvas1 = tk.Canvas(root, width=300, height=300, bg='lightsteelblue')
canvas1.pack()

mag_keys = list()
k = ''
sp = list()
p_on = list()
m_num = 0


def getExcel():

    import_file_path = filedialog.askopenfilename()
    param_df = pd.read_csv(
        import_file_path, usecols=['P_names', 'Params'])

    param_key = param_df['P_names']
    params = param_df['Params']

    for ind, key in enumerate(param_key):
        if key == 'NaN':
            pass
        else:
            param_dict[key] = params[ind]

    m_num = int(param_dict['number_of_magnets'])

    for n in range(0, m_num):
        k = 'M'+str(n)
        mag_keys.append(k)

    sp_df = pd.read_csv(
        import_file_path, usecols=mag_keys)

    D3 = sp_df.replace(np.nan, 0)

    D4 = sp_df.replace(np.nan, False)
    D5 = D4.mask(D4 != False, True)

    reject_data = np.zeros((1, m_num))

    sp_len = sp_df.shape[0]
    for ind in range(0, sp_len):
        sp_row = D3.iloc[[ind]]
        sp_check = sp_row.to_numpy()

        p_on_df = D5.iloc[[ind]]
        p_on_np = p_on_df.to_numpy()
        disc = (sp_check[0] == reject_data[0])

        if disc.all():
            pass
        else:
            sp = sp_check[0]
            setpoints.append(sp)
            p_on = p_on_np[0]
            pulse_on.append(p_on)

    root.destroy()
    return setpoints, param_dict, pulse_on


browseButton_Excel = tk.Button(text='Import Excel File', command=getExcel,
                               bg='green', fg='white', font=('helvetica', 12, 'bold'))
canvas1.create_window(150, 150, window=browseButton_Excel)
root.mainloop()

#----------function to pulse and store data from magnets--------


def pulseMagnet(param_dict, setpoints, pulse_on, abs_pulse=50, loop_iteration_total=10, save_data=True):

    magnets_num = int(param_dict['number_of_magnets'])
    Kp = param_dict['Kp_init']
    Ki = param_dict['Ki_init']
    beta = param_dict['Beta']

    min_pulse_length, max_pulse_length = -abs_pulse, abs_pulse
    
    magnets = []
    controllers = []



    for n in range(0,magnets_num):   
        controllers.append(FeedbackController(Kp,Ki, Kd, 
                                            beta, 
                                            setpoint=setpoints, 
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
    
            print("\tHall Sensor [",sen_address[n],"]","=",zReading_Sensor_Average,"Gauss","==> Pulse:",round(u,2),"us")
    
        nmrObj.igbtPulseMagnetControl(plength, pspac, pulse_reptition) # actuate magnets
        print("\t plength : {}".format(plength))
    
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


# -------initialize magnets----------------
p_on = pulse_on[0]
abs_pulse = 50
pulseMagnet(param_dict, setpoints[0], p_on, abs_pulse,
            loop_iteration_total=10, save_data=True)


#---------------setup experiment here-----------------------------------

setpoints_expt = setpoints[1:]
pulse_on_expt = pulse_on[1:]

for ind, sp in enumerate(setpoints_expt):
    p_on = pulse_on_expt[ind]
    abs_pulse = 50
    pulseMagnet(param_dict, sp, p_on, abs_pulse,
                loop_iteration_total=10, save_data=True)

    for pl in abs_pl:
        pulseMagnet(param_dict, 2000, p_on, pl,
                    loop_iteration_total=2, save_data=True)

        pulseMagnet(param_dict, sp, p_on, abs_pulse,
                    loop_iteration_total=10, save_data=True)
        
        
        
        
# plot results
# if plot_results == True:
#     t = np.linspace(0,controllers[0].time_total,controllers[0].time_total)
#     
#     rows, columns = 3,3
#     fig,a = plt.subplots(rows,columns)
#     fig,b = plt.subplots(rows,columns)
#     
#     k = 0
#     for n in range(0,rows): # rows
#         for m in range(0,columns): # columns
#           a[n][m].set_title('Magnet '+ str(sen_address[k]))
#           #a[n][m].set_xlabel('Iterations')
#           a[n][m].set_ylabel('Magnetization (Gauss)')
#           a[n][m].plot(t,np.ones(controllers[k].time_total)*setpoints[k],'r-', label='setpoint')  
#           a[n][m].plot(t,controllers[k].input_data,'b-', label='magnetization')  
#           #a[n][m].plot(t,controllers[k].error_data,'y-', label='error')
#           a[n][m].grid(True)
#           if k == 0:
#               a[n][m].legend(loc='best')
#           
#           b[n][m].set_title('Magnet '+ str(sen_address[k]))
#           #b[n][m].set_xlabel('Iterations')
#           b[n][m].set_ylabel('Pulse (us)')
#           b[n][m].plot(t,np.ones(controllers[k].time_total)*max_pulse_length,'r--', label='max_pulse') 
#           b[n][m].plot(t,np.ones(controllers[k].time_total)*min_pulse_length,'b--', label='min_pulse') 
#           b[n][m].plot(t,controllers[k].output_data,'g*-', label='pulse')
#           b[n][m].grid(True)
#           if k == 0:
#               b[n][m].legend(loc='best')
#         
#           k += 1
#     plt.show()
    
    #a.imsave(data_folder + '/magnetization ' + st, format = 'png')
    #b.imsave(data_folder + '/actuation ' + st, format = 'png')
    
    
    
