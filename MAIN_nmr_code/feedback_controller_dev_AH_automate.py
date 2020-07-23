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
from nmr_std_function.feedback_controller_class_004_AH import *

save_data = True
plot_results = True
# Default: Feedback_Control_Results_
logging_file_name = 'Exp 2 large Kp value setpoint -500 '

# variables
data_folder = "/root/HallSensorData"
en_remote_dbg = 0
nmrObj = tunable_nmr_system_2018(data_folder, en_remote_dbg)

try:
    # delete current_reading.csv every time
    os.remove(data_folder + '/Current_Reading.csv')
except:
    print("file does not exist")


# --------------------- Pin address assignement --------------------------------
num_channel = 36
pspac = 200             # maximum pulse length
pulse_reptition = 1     # No of pulses in one sequence
plength = np.zeros(num_channel)

#plength_forward = numpy.zeros(num_channel)
#plength_backward = numpy.zeros(num_channel)
n_reading = 1   # number of reading

sen_address = [28, 29, 31, 33, 32, 30, 34, 35, 36]

forward_direction_addr = [0, 4, 1, 5, 8, 12, 9, 13, 16]
reverse_direction_addr = [2, 6, 3, 7, 10, 14, 11, 15, 18]

enable_message = False
if enable_message == False:
    print("\n")
    print("\t Message Disabled")
    print("\n")
# ------------------------------------------------------------------------------


# -------------------- Controller initialization -------------------------------
loop_iteration_total = [1, 10]
abs_pl = range(5, 55, 5)
setpoints = []
pulse_on = []
param_dict = dict()

root = tk.Tk()
canvas1 = tk.Canvas(root, width=300, height=300, bg='lightsteelblue')
canvas1.pack()


sp_final = []
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


def pulseMagnet(param_dict, setpoints, pulse_on, abs_pulse, loop_iteration_total=10, save_data=True):

    magnets_num = int(param_dict['number_of_magnets'])
    Kp = param_dict['Kp_init']
    Ki = param_dict['Ki_init']
    beta = param_dict['Beta']

    min_pulse_length, max_pulse_length = -abs_pulse, abs_pulse

    bh_factor = np.zeros(magnets_num)
    v1 = 1
    v2 = 1.5
    v3 = 3

    bh_factor[0] = v1
    bh_factor[1] = v2
    bh_factor[2] = v1
    bh_factor[3] = v2
    bh_factor[4] = v3
    bh_factor[5] = v2
    bh_factor[6] = v1
    bh_factor[7] = v2
    bh_factor[8] = v1

    for n in range(0, magnets_num):
        Kp_weight = Kp * bh_factor[n]
        Ki_weight = Ki * bh_factor[n]

        controllers.append(FeedbackController(Kp_weight, Ki_weight, Kd, beta,
                                              setpoint=setpoints[n],
                                              output_limits=(
                                                  min_pulse_length, max_pulse_length),
                                              setpoint_weighting=False,
                                              proportional_on_input=False,
                                              output_enabled=pulse_on[n],
                                              measurement_smoothing_enabled=False,
                                              measurement_smoothing_start=10,
                                              measurement_smoothing_past=3))

# ------------------------------------------------------------------------------

    for t in range(0, loop_iteration_total):
        plength = np.zeros(num_channel)

        print("Start of iteration {}".format(t))
        # read hall sensor value in Tesla
        nmrObj.igbtSenReadingMulti(sen_address, n_reading, enable_message)
        zReading = parse_csv_returnZreading(
            data_folder, 'Current_Reading.csv')  # in Gauss

        for n in range(0, magnets_num):
            # read hall sensor value in Tesla
            # nmrObj.igbtSenReading(sen_address[n], n_reading)

            zReading_Sensor = zReading[n_reading * n: n_reading*(n+1)]

            # kalman filter
            ZReading_Average = Kalman_Filter(n_reading, zReading_Sensor)
            y = ZReading_Average

            # average past measurements to reduce sensor noise
            t_past = 3
            if t > t_past:
                temp = np.mean(controllers[n].input_data[t-t_past:t])
                past_average = np.mean([temp, y])
                y = round(past_average, 2)

            # print("\n")
            print("\tCurrent hall reading is : {}".format(y))
            print("\tSensor Address is : {}".format(sen_address[n]))
            # print("\n")

            # ---------------------------
            # compute new ouput. Save value in array U
            u = controllers[n].update(y)
            # determine by the pin assignment to the channels
            u = u * (-1)

            if u >= 0:
                plength[forward_direction_addr[n]] = abs(
                    np.int(u))                         # actuate magnets

            if u < 0:
                plength[reverse_direction_addr[n]] = abs(np.int(u))

        # delete current_reading.csv every time
        os.remove(data_folder + '/Current_Reading.csv')
        print("\t plength : {}".format(plength))
        nmrObj.igbtPulseMagnetControl(plength, pspac, pulse_reptition)

    # save results as MAT file

    if save_data == True:
        experiment_data = {}
        st = datetime.datetime.fromtimestamp(
            time.time()).strftime('%Y_%m_%d %H_%M_%S')
        for n in range(0, magnets_num):
            experiment_data.update({'magnet_'+str(n): {
                'control_tunings': controllers[n].tunings,
                'beta': controllers[n].beta,
                'setpoint_weighting': controllers[n].setpoint_weighting,
                'proportional_on_input': controllers[n].proportional_on_input,
                'Iteration': controllers[n].time_total,
                'control_pulse': controllers[n].output_data,
                'magnetization': controllers[n].input_data,
                'error': controllers[n].error_data,
                'setpoint': controllers[n].setpoint,
                'integral': controllers[n].integral_data
            }})

    sio.savemat(logging_file_name+st+'.mat', experiment_data)


# -------initialize magnets----------------
p_on = pulse_on[0]
abs_pulse = 50
pulseMagnet(param_dict, setpoints[0], p_on, abs_pulse,
            loop_iteration_total=10, save_data=True)

setpoints_expt = setpoints[1:]
pulse_on_expt = pulse_on[1:]

for ind, sp in enumerate(setpoints_expt):
    p_on = pulse_on_expt[ind]
    abs_pulse = 50
    pulseMagnet(param_dict, sp, p_on, abs_pulse,
                loop_iteration_total=10, save_data=True)

    for pl in abs_pl:
        pulseMagnet(param_dict, 2000, p_on, pl,
                    loop_iteration_total=1, save_data=True)

        pulseMagnet(param_dict, sp, p_on, abs_pulse,
                    loop_iteration_total=10, save_data=True)
