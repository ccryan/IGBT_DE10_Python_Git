'''
Created on July 10, 2020
Updated on July 29, 2020
Verification: July 29, 2020

Author: Roland Probst

Updates:
- Doing: create separate file for IO operations, e.g. hall sensor read, magnet actuation 
- Done: Magnet response measurement for model based controller
'''

import os
import matplotlib.pyplot as plt
import scipy.io as sio
import numpy as np
import datetime
import time
import socket

from nmr_std_function.feedback_controller_io import *
from nmr_std_function.feedback_controller_class_revision005 import *

# -------------------- IGBT and hall sensor io initialization -------------------------------

control_io = feedback_controller_io(
	num_channels=36,
	max_pulse_length_hardware_limit=200,							# maximum pulse length
	pulse_repetition=1, 											# number of pulses in one sequence
	hall_sensor_address=[19, 20, 21, 22, 23 ,24, 25, 26, 27],			# Pin address assignment
	actuation_forward_direction_address=[22, 19, 23, 26, 30, 27, 31, 34, 35],
	actuation_reverse_direction_address=[20, 17, 21, 24, 28, 25, 29, 32, 33],
	hall_sensor_reading_repetition=1,
	data_folder="/root/HallSensorData")

# -------------------- Controller initialization -------------------------------
magnets_num = 9
magnets = []
controllers = []

controller_mode = "model_based"  # PID, model_based, 
feedback_loop_iteration_total = 50  # control loop iterations
hall_sensor_reading_repetition = 1  # hall sensor readings for Kalman Filter

save_data = True
plot_results = True
logging_file_name = 'Feedback_Control_Results_'

Kp = 15  # proportional gain - needs tuning
Ki = 15  # integral gain - needs tuning
Kd = 0  # differential gain
beta = 0.8  # setpoint weighting - needs tuning

min_pulse_length, max_pulse_length = -50, 50.  # pulse length limits in mico seconds

setpoints = np.zeros(magnets_num)  # target magnetization in Gauss
value = 30  # Note:avoid multiples of 64
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


# Magnet pulse response curve
'''
magnetization_curve_pulse = 10
magnetization_curve_forward = [-636.0, -476.0, -348.0, -248.0, -156.0, -68.0, 12.0, 84.0, 144.0, 204.0, 256.0, 308.0, 356.0, 396.0, 432.0, 464.0, 500.0, 528.0, 552.0, 576.0, 596.0, 616.0, 628.0, 640.0, 656.0, 672.0, 680.0, 688.0, 696.0, 708.0, 712.0]
magnetization_curve_reverse = [724.0, 560.0, 440.0, 340.0, 248.0, 164.0, 84.0, 4.0, -52.0, -112.0, -164.0, -216.0, -256.0, -296.0, -328.0, -360.0, -392.0, -416.0, -440.0, -460.0, -484.0, -500.0, -520.0, -536.0, -548.0, -556.0, -572.0, -584.0, -592.0, -600.0, -604.0, -616.0, -624.0, -628.0, -632.0, -644.0, -652.0, -656.0, -660.0, -664.0, -664.0, -672.0, -676.0, -676.0, -680.0, -684.0, -688.0, -688.0, -688.0]
'''
magnetization_curve_pulse = 50
magnetization_curve_forward = [28.0, 268.0, 464.0, 636.0, 780.0, 916.0, 1016.0, 1076.0, 1096.0, 1112.0, 1116.0, 1120.0, 1120.0, 1120.0, 1120.0, 1124.0, 1120.0, 1124.0, 1124.0, 1120.0, 1120.0, 1120.0, 1124.0, 1120.0, 1120.0, 1116.0, 1120.0, 1124.0, 1124.0, 1116.0, 1128.0, 1124.0, 1124.0, 1124.0, 1120.0, 1124.0, 1124.0, 1124.0, 1120.0, 1120.0, 1120.0, 1124.0, 1124.0, 1124.0, 1124.0, 1120.0, 1120.0, 1124.0, 1120.0, 1128.0] 
magnetization_curve_reverse = [-32.0, -304.0, -528.0, -716.0, -880.0, -1088.0, -1124.0, -1172.0, -1180.0, -1184.0, -1184.0, -1184.0, -1188.0, -1188.0, -1188.0, -1188.0, -1188.0, -1188.0, -1188.0, -1192.0, -1188.0, -1188.0, -1188.0, -1188.0, -1188.0, -1188.0, -1188.0, -1188.0, -1188.0, -1192.0, -1188.0, -1192.0, -1188.0, -1188.0, -1192.0, -1188.0, -1188.0, -1192.0, -1188.0, -1192.0, -1192.0, -1188.0, -1188.0, -1188.0, -1192.0, -1192.0, -1192.0, -1188.0, -1192.0, -1192.0]

for n in range(0, magnets_num):
    controllers.append(FeedbackController(Kp, Ki, Kd,
                                          beta,
                                          setpoint=setpoints[n],
                                          output_limits=(min_pulse_length, max_pulse_length),
                                          setpoint_weighting=False,
                                          proportional_on_input=False,
                                          output_enabled=pulse_on[n],
                                          magnetization_curve_forward=magnetization_curve_forward,
                                          magnetization_curve_reverse=magnetization_curve_reverse))
										  
# ------------------------------------------------------------------------------

for t in range(0, feedback_loop_iteration_total):

	print("Start of iteration ", t, "/", feedback_loop_iteration_total)
	
    # read all hall sensor values in Gauss
	hall_sensor_array_reading = control_io.read_hall_sensor_array()
	
	actuation_vector=np.zeros(control_io.num_channels)
	
	for n in range(0, magnets_num):
    	# get hall sensor value for magnet n
		hall_sensor_value = control_io.get_hall_sensor_value_z(hall_sensor_array_reading,n)
        
        # compute new output
		if controller_mode == "PID":
			u = controllers[n].update(hall_sensor_value) # PID controller
		elif controller_mode == "model_based":
			u = controllers[n].update_model_based(hall_sensor_value) # model based controller
        
		# assemble actuation vector
		if u >= 0:
			actuation_vector[control_io.actuation_forward_direction_address[n]] = abs(np.int(u))
		if u < 0:
			actuation_vector[control_io.actuation_reverse_direction_address[n]] = abs(np.int(u))
			
		print("\t Hall Sensor [", control_io.hall_sensor_address[n], "]", "=", hall_sensor_value, "Gauss", "==> Pulse:", round(u, 2),"us")
    
	# actuate all magnets
	control_io.pulse_all(actuation_vector)

	       
# save results as MAT file
if save_data == True:
    experiment_data = {}
    st = datetime.datetime.fromtimestamp(time.time()).strftime('%Y_%m_%d %H_%M_%S')
    for n in range(0, magnets_num):
        experiment_data.update({'magnet_' + str(n): {
            'control_tunings': controllers[n].tunings,
            'beta': controllers[n].beta,
            'setpoint_weighting': controllers[n].setpoint_weighting,
            'proportional_on_input': controllers[n].proportional_on_input,
            'Iteration': controllers[n].time_total,
            'control_pulse': controllers[n].output_data,
            'magnetization': controllers[n].input_data,
            'error': controllers[n].error_data,
            'setpoint': controllers[n].setpoint,
            'integral': controllers[n].integral_data,
            'magnetization_curve_forward':magnetization_curve_forward,
            'magnetization_curve_reverse':magnetization_curve_reverse,
            'magnetization_curve_pulse':magnetization_curve_pulse
        }})

    sio.savemat(logging_file_name + st + '.mat', experiment_data)

# plot results
if plot_results == True:
    t = np.linspace(0, controllers[0].time_total, controllers[0].time_total)

    rows, columns = 3, 3
    fig, a = plt.subplots(rows, columns)
    fig, b = plt.subplots(rows, columns)

    k = 0
    for n in range(0, rows):  # rows
        for m in range(0, columns):  # columns
            a[n][m].set_title('Magnet ' + str(sen_address[k]))
            # a[n][m].set_xlabel('Iterations')
            a[n][m].set_ylabel('Magnetization (Gauss)')
            a[n][m].plot(t, np.ones(controllers[k].time_total) * setpoints[k], 'r-', label='setpoint')
            a[n][m].plot(t, controllers[k].input_data, 'b-', label='magnetization')
            # a[n][m].plot(t,controllers[k].error_data,'y-', label='error')
            a[n][m].grid(True)
            if k == 0:
                a[n][m].legend(loc='best')

            b[n][m].set_title('Magnet ' + str(sen_address[k]))
            # b[n][m].set_xlabel('Iterations')
            b[n][m].set_ylabel('Pulse (us)')
            b[n][m].plot(t, np.ones(controllers[k].time_total) * max_pulse_length, 'r--', label='max_pulse')
            b[n][m].plot(t, np.ones(controllers[k].time_total) * min_pulse_length, 'b--', label='min_pulse')
            b[n][m].plot(t, controllers[k].output_data, 'g*-', label='pulse')
            b[n][m].grid(True)
            if k == 0:
                b[n][m].legend(loc='best')

            k += 1
    plt.show()