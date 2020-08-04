'''
Created on July 27, 2020
Updated on July 27, 2020
Verification: July 27, 2020

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


class feedback_controller_io(object):
	"""
	Input output functions for feedback controller
	"""
	def __init__(
		self,
		num_channels=36,
		max_pulse_length_hardware_limit=200,							# maximum pulse length
		pulse_repetition=1, 							# number of pulses in one sequence
		hall_sensor_address=[28, 29, 31, 33, 32, 30, 34, 35, 36],			# Pin address assignment
		actuation_forward_direction_address=[2, 6, 3, 7, 10, 14, 11, 15, 18],
		actuation_reverse_direction_address=[0, 4, 1, 5, 8, 12, 9, 13, 16],
		hall_sensor_reading_repetition=1,
		data_folder="/root/HallSensorData"
	):
		self.num_channels = num_channels
		self.max_pulse_length_hardware_limit=max_pulse_length_hardware_limit
		self.pulse_repetition=pulse_repetition
		self.actuation_vector=np.zeros(self.num_channels)
		self.hall_sensor_address=hall_sensor_address
		self.actuation_forward_direction_address = actuation_forward_direction_address
		self.actuation_reverse_direction_address = actuation_reverse_direction_address
		self.hall_sensor_reading_repetition = hall_sensor_reading_repetition
		
		# data folder 
		self.data_folder = data_folder
		en_remote_dbg = 0
		self.nmrObj = tunable_nmr_system_2018(self.data_folder, en_remote_dbg)
		
	  
	def delete_current_hall_sensor_reading(self):
		try:
			os.remove(self.data_folder + '/Current_Reading.csv')  # delete current_reading.csv 
		except:
			print("file does not exist")
		
	def read_hall_sensor_array(self):
		enable_message = False
		self.nmrObj.igbtSenReadingMulti(self.hall_sensor_address, self.hall_sensor_reading_repetition, enable_message)
		zReading = parse_csv_returnZreading(self.data_folder, 'Current_Reading.csv')
		
		return zReading
	
	def get_hall_sensor_value_z(self,zReading,sensor_idx):
		zReading_Sensor = zReading[self.hall_sensor_reading_repetition * sensor_idx: self.hall_sensor_reading_repetition * (sensor_idx + 1)]
		# Kalman filter for hall sensor noise reduction
		zReading_Sensor_Average = Kalman_Filter(self.hall_sensor_reading_repetition, zReading_Sensor)
	
		return zReading_Sensor_Average
	
	def pulse_single_forward(self,pulse,magnet_idx):
		self.actuation_vector = np.zeros(self.num_channels) # clear and initialize pulse vector
		self.actuation_vector[self.actuation_forward_direction_address[magnet_idx]] = abs(np.int(pulse))
		self.nmrObj.igbtPulseMagnetControl(self.actuation_vector, self.max_pulse_length_hardware_limit, self.pulse_repetition)  # actuate magnets
		print("\t IGBT Pulse : {}".format(self.actuation_vector))
		
	def pulse_single_reverse(self,pulse,magnet_idx):
		self.actuation_vector = np.zeros(self.num_channels) # clear and initialize pulse vector
		self.actuation_vector[self.actuation_reverse_direction_address[magnet_idx]] = abs(np.int(pulse))
		self.nmrObj.igbtPulseMagnetControl(self.actuation_vector, self.max_pulse_length_hardware_limit, self.pulse_repetition)  # actuate magnets
		print("\t IGBT Pulse : {}".format(self.actuation_vector))
		
	def pulse_all(self,pulse_vector):
		self.actuation_vector = np.zeros(self.num_channels) # clear and initialize pulse vector
		self.actuation_vector = pulse_vector
		self.nmrObj.igbtPulseMagnetControl(self.actuation_vector, self.max_pulse_length_hardware_limit, self.pulse_repetition)  # actuate magnets
		print("\t IGBT Pulse : {}".format(self.actuation_vector))
        
	def pulse_all_forward(self,pulse=0,repetition=1):
		for n in range(0,repetition):
			self.actuation_vector = np.zeros(self.num_channels) # clear and initialize pulse vector
			self.actuation_vector[self.actuation_forward_direction_address] = abs(np.int(pulse))
			self.nmrObj.igbtPulseMagnetControl(self.actuation_vector, self.max_pulse_length_hardware_limit, self.pulse_repetition)  # actuate magnets
			print("\t IGBT Forward Pulse : {}".format(self.actuation_vector))	
		
	def pulse_all_reverse(self,pulse=0,repetition=1):
		for n in range(0,repetition):
			self.actuation_vector = np.zeros(self.num_channels) # clear and initialize pulse vector
			self.actuation_vector[self.actuation_reverse_direction_address] = abs(np.int(pulse))
			self.nmrObj.igbtPulseMagnetControl(self.actuation_vector, self.max_pulse_length_hardware_limit, self.pulse_repetition)  # actuate magnets
			print("\t IGBT Reverse Pulse : {}".format(self.actuation_vector))
		
  
  
  