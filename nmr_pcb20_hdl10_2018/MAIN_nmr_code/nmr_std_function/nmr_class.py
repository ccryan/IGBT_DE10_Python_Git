'''
Created on Nov 06, 2018

@author: David Ariando

2020-04-15, Cheng: Add IGBT pulse control (igbtPulse)
'''

#!/usr/bin/python

import os
import pydevd
import numpy as np
import matplotlib.pyplot as plt
from nmr_std_function.data_parser import parse_simple_info
from nmr_std_function.nmr_functions import compute_iterate
from nmr_std_function.data_parser import parse_csv_float2col
from scipy import signal
from datetime import datetime
import shutil
from nmr_std_function.data_parser import write_text_append
from nmr_std_function.hw_driver import fpga_de1soc


class tunable_nmr_system_2018:
    def __init__(self, data_folder, en_remote_dbg):
        self.PCBVer = 'v5.0'  # options are v4.0_and_below, v5.0

        if self.PCBVer == 'v4.0_and_below':
            # Output control signal to FPGA via I2C
            self.PSU_15V_TX_P_EN_ofst = (0)
            self.PSU_15V_TX_N_EN_ofst = (1)
            self.AMP_HP_LT1210_EN_ofst = (2)
            self.PSU_5V_TX_N_EN_ofst = (3)
            self.PAMP_IN_SEL_TEST_ofst = (4)
            self.PAMP_IN_SEL_RX_ofst = (5)
            self.GPIO_GEN_PURP_1_ofst = (6)
            self.PSU_5V_ADC_EN_ofst = (7)
            self.RX_AMP_GAIN_2_ofst = (8)
            self.RX_AMP_GAIN_1_ofst = (9)
            self.RX_AMP_GAIN_4_ofst = (10)
            self.RX_AMP_GAIN_3_ofst = (11)
            self.RX_IN_SEL_1_ofst = (12)
            self.RX_IN_SEL_2_ofst = (13)
            self.PSU_5V_ANA_P_EN_ofst = (14)
            self.PSU_5V_ANA_N_EN_ofst = (15)
            # Output control signal mask to FPGA via I2C
            self.PSU_15V_TX_P_EN_msk = (1 << self.PSU_15V_TX_P_EN_ofst)
            self.PSU_15V_TX_N_EN_msk = (1 << self.PSU_15V_TX_N_EN_ofst)
            self.AMP_HP_LT1210_EN_msk = (1 << self.AMP_HP_LT1210_EN_ofst)
            self.PSU_5V_TX_N_EN_msk = (1 << self.PSU_5V_TX_N_EN_ofst)
            self.PAMP_IN_SEL_TEST_msk = (1 << self.PAMP_IN_SEL_TEST_ofst)
            self.PAMP_IN_SEL_RX_msk = (1 << self.PAMP_IN_SEL_RX_ofst)
            self.GPIO_GEN_PURP_1_msk = (1 << self.GPIO_GEN_PURP_1_ofst)
            self.PSU_5V_ADC_EN_msk = (1 << self.PSU_5V_ADC_EN_ofst)
            self.RX_AMP_GAIN_2_msk = (1 << self.RX_AMP_GAIN_2_ofst)
            self.RX_AMP_GAIN_1_msk = (1 << self.RX_AMP_GAIN_1_ofst)
            self.RX_AMP_GAIN_4_msk = (1 << self.RX_AMP_GAIN_4_ofst)
            self.RX_AMP_GAIN_3_msk = (1 << self.RX_AMP_GAIN_3_ofst)
            self.RX_IN_SEL_1_msk = (1 << self.RX_IN_SEL_1_ofst)
            self.RX_IN_SEL_2_msk = (1 << self.RX_IN_SEL_2_ofst)
            self.PSU_5V_ANA_P_EN_msk = (1 << self.PSU_5V_ANA_P_EN_ofst)
            self.PSU_5V_ANA_N_EN_msk = (1 << self.PSU_5V_ANA_N_EN_ofst)

        elif self.PCBVer == 'v5.0':
            # chip offset declaration (look at the KiCAD for the designated
            # chip name)
            self.i2c_U21_ofst = 0
            self.i2c_U71_ofst = 16
            self.spi_pamp_U32 = 32

            # Output control signal to FPGA via I2C addr:0x40
            self.RX_FL_ofst = (0 + self.i2c_U21_ofst)
            self.RX_FH_ofst = (1 + self.i2c_U21_ofst)
            self.RX_SEL2_ofst = (2 + self.i2c_U21_ofst)
            self.RX_SEL1_ofst = (3 + self.i2c_U21_ofst)
            self.RX3_L_ofst = (4 + self.i2c_U21_ofst)
            self.RX3_H_ofst = (5 + self.i2c_U21_ofst)
            self.RX1_2L_ofst = (6 + self.i2c_U21_ofst)
            self.RX1_2H_ofst = (7 + self.i2c_U21_ofst)
            # self.___(8 + self.i2c_U21_ofst)
            # self.___(9 + self.i2c_U21_ofst)
            # self.___(10 + self.i2c_U21_ofst)
            self.PAMP_RDY_ofst = (11 + self.i2c_U21_ofst)
            self.RX1_1H_ofst = (12 + self.i2c_U21_ofst)
            self.RX1_1L_ofst = (13 + self.i2c_U21_ofst)
            self.RX2_H_ofst = (14 + self.i2c_U21_ofst)
            self.RX2_L_ofst = (15 + self.i2c_U21_ofst)

            self.RX_FL_msk = (1 << self.RX_FL_ofst)
            self.RX_FH_msk = (1 << self.RX_FH_ofst)
            self.RX_SEL2_msk = (1 << self.RX_SEL2_ofst)
            self.RX_SEL1_msk = (1 << self.RX_SEL1_ofst)
            self.RX3_L_msk = (1 << self.RX3_L_ofst)
            self.RX3_H_msk = (1 << self.RX3_H_ofst)
            self.RX1_2L_msk = (1 << self.RX1_2L_ofst)
            self.RX1_2H_msk = (1 << self.RX1_2H_ofst)
            # self.___(8)
            # self.___(9)
            # self.___(10)
            self.PAMP_RDY_msk = (1 << self.PAMP_RDY_ofst)
            self.RX1_1H_msk = (1 << self.RX1_1H_ofst)
            self.RX1_1L_msk = (1 << self.RX1_1L_ofst)
            self.RX2_H_msk = (1 << self.RX2_H_ofst)
            self.RX2_L_msk = (1 << self.RX2_L_ofst)

            # Output control signal to FPGA via I2C addr:0x41
            # self.___(0 + self.i2c_U71_ofst)
            # self.___(1 + self.i2c_U71_ofst)
            # self.___(2 + self.i2c_U71_ofst)
            # self.___(3 + self.i2c_U71_ofst)
            # self.___(4 + self.i2c_U71_ofst)
            # self.___(5 + self.i2c_U71_ofst)
            self.DUP_STAT_ofst = (6 + self.i2c_U71_ofst)
            self.QSW_STAT_ofst = (7 + self.i2c_U71_ofst)
            self.PSU_5V_ADC_EN_ofst = (8 + self.i2c_U71_ofst)
            self.PSU_5V_ANA_N_EN_ofst = (9 + self.i2c_U71_ofst)
            self.PSU_5V_ANA_P_EN_ofst = (10 + self.i2c_U71_ofst)
            self.MTCH_NTWRK_RST_ofst = (11 + self.i2c_U71_ofst)
            self.PSU_15V_TX_P_EN_ofst = (12 + self.i2c_U71_ofst)
            self.PSU_15V_TX_N_EN_ofst = (13 + self.i2c_U71_ofst)
            self.PSU_5V_TX_N_EN_ofst = (14 + self.i2c_U71_ofst)
            # self.___(15 + self.i2c_U71_ofst)
            # self.___(0 + 16)
            # self.___(1 + 16)
            # self.___(2 + 16)
            # self.___(3 + 16)
            # self.___(4 + 16)
            # self.___(5 + 16)
            self.DUP_STAT_msk = (1 << self.DUP_STAT_ofst)
            self.QSW_STAT_msk = (1 << self.QSW_STAT_ofst)
            self.PSU_5V_ADC_EN_msk = (1 << self.PSU_5V_ADC_EN_ofst)
            self.PSU_5V_ANA_N_EN_msk = (1 << self.PSU_5V_ANA_N_EN_ofst)
            self.PSU_5V_ANA_P_EN_msk = (1 << self.PSU_5V_ANA_P_EN_ofst)
            self.MTCH_NTWRK_RST_msk = (1 << self.MTCH_NTWRK_RST_ofst)
            self.PSU_15V_TX_P_EN_msk = (1 << self.PSU_15V_TX_P_EN_ofst)
            self.PSU_15V_TX_N_EN_msk = (1 << self.PSU_15V_TX_N_EN_ofst)
            self.PSU_5V_TX_N_EN_msk = (1 << self.PSU_5V_TX_N_EN_ofst)
            # self.___(15 + 16)

            # definition for spi pamp input control
            self.PAMP_IN_SEL1_ofst = (2 + self.spi_pamp_U32)
            self.PAMP_IN_SEL2_ofst = (3 + self.spi_pamp_U32)
            self.PAMP_IN_SEL1_msk = (1 << self.PAMP_IN_SEL1_ofst)
            self.PAMP_IN_SEL2_msk = (1 << self.PAMP_IN_SEL2_ofst)

        # General control defaults for the FPGA
        self.gnrl_cnt = 0

        # ip addresses settings for the system
        self.server_ip = '192.168.137.2'  # '129.22.143.88'
        self.client_ip = '192.168.137.1'  # '129.22.143.39'
        self.server_path = '/root/nmr_pcb20_hdl10_2018/MAIN_nmr_code/'
        # client path with samba
        self.client_path = 'X:\\nmr_pcb20_hdl10_2018\\MAIN_nmr_code\\'

        if en_remote_dbg:
            from pydevd_file_utils import setup_client_server_paths
            PATH_TRANSLATION = [(self.client_path, self.server_path)]
            setup_client_server_paths(PATH_TRANSLATION)
            print("---server:%s---client:%s---" %
                  (self.server_ip, self.client_ip))
            pydevd.settrace(self.client_ip, stdoutToServer=True,
                            stderrToServer=True)

        # variables
        self.data_folder = data_folder
        self.exec_folder = "/c_exec/"

        # directories
        self.work_dir = os.getcwd()
        # only do this after remote debug initialization
        os.chdir(self.data_folder)

    def turnOnRemoteDebug(self):
        # CANNOT RUN FROM HERE, YOU HAVE TO COPY THE CONTENT OF THIS FOLDER TO THE EXECUTABLE WHERE YOU RUN THE CODE AND RUN IT FROM THERE
        # THIS IS KEPT FOR CLEAN DOCUMENTATION PURPOSES
        #from pydevd_file_utils import setup_client_server_paths
        #server_path = '/root/nmr_pcb20_hdl10_2018/MAIN_nmr_code/'
        #client_path = 'D:\\GDrive\\WORKSPACES\\Eclipse_Python_2018\\RemoteSystemsTempFiles\\DAJO-DE1SOC\\root\\nmr_pcb20_hdl10_2018\\MAIN_nmr_code\\'
        #PATH_TRANSLATION = [(client_path, server_path)]
        # setup_client_server_paths(PATH_TRANSLATION)
        # pydevd.settrace("dajo-compaqsff")
        from pydevd_file_utils import setup_client_server_paths
        PATH_TRANSLATION = [(self.client_path, self.server_path)]
        setup_client_server_paths(PATH_TRANSLATION)
        print("---server:%s---client:%s---" %
              (self.server_ip, self.client_ip))
        pydevd.settrace(self.client_ip, stdoutToServer=True,
                        stderrToServer=True)

    def initNmrSystem(self):
        os.system(self.work_dir + "/c_exec/init")

    def setPreampTuning(self, vbias, vvarac):
        # set preamp tuning
        os.system(
            self.work_dir + self.exec_folder + "preamp_tuning" + " " +
            str(vbias) + " " +
            str(vvarac)
        )

    def setMatchingNetwork(self, cpar, cser):
        if self.PCBVer == 'v5.0':
            self.assertControlSignal(self.MTCH_NTWRK_RST_msk)

        # Turn on matching network
        # self.cshunt = cpar
        # self.cseries = cser
        os.system(
            self.work_dir + self.exec_folder + "i2c_mtch_ntwrk" + " " +
            str(cpar) + " " +
            str(cser)
        )

    def setSignalPath(self):  # ONLY VALID FOR v4.0 and below
        # activate transmitter and stuffs
        self.gnrl_cnt = self.gnrl_cnt | self.AMP_HP_LT1210_EN_msk | self.PAMP_IN_SEL_RX_msk | self.RX_IN_SEL_1_msk
        os.system(
            self.work_dir + self.exec_folder + "i2c_gnrl" + " " +
            str(self.gnrl_cnt)
        )

    def turnOnPower(self):  # ONLY VALID FOR v4.0 and below
        # Turn on power supply
        self.gnrl_cnt = self.PSU_15V_TX_P_EN_msk | self.PSU_15V_TX_N_EN_msk | self.PSU_5V_TX_N_EN_msk | self.PSU_5V_ADC_EN_msk | self.PSU_5V_ANA_P_EN_msk | self.PSU_5V_ANA_N_EN_msk
        os.system(
            self.work_dir + self.exec_folder + "i2c_gnrl" + " " +
            str(self.gnrl_cnt)
        )

    def turnOffSystem(self):  # ONLY VALID FOR v4.0 and below
        # Turn off matching network
        os.system(
            self.work_dir + self.exec_folder + "i2c_mtch_ntwrk" + " " +
            str(0) + " " +
            str(0)
        )
        # Disable all power and path
        os.system(
            self.work_dir + self.exec_folder + "i2c_gnrl" + " " +
            str(0)
        )

    def assertControlSignal(self, cnt_in):
        self.gnrl_cnt = self.gnrl_cnt | cnt_in

        if self.PCBVer == 'v4.0_and_below':
            os.system(
                self.work_dir + self.exec_folder + "i2c_gnrl" + " " +
                str(self.gnrl_cnt)
            )
        elif self.PCBVer == 'v5.0':
            self.gnrl_cnt0 = (self.gnrl_cnt >> self.i2c_U21_ofst) & 0xffff
            self.gnrl_cnt1 = (self.gnrl_cnt >> self.i2c_U71_ofst) & 0xffff
            self.gnrl_cnt2 = (self.gnrl_cnt >> self.spi_pamp_U32) & 0xff
            os.system(
                self.work_dir + self.exec_folder + "i2c_gnrl" + " " +
                str(self.gnrl_cnt0) + " " +
                str(self.gnrl_cnt1)
            )
            os.system(self.work_dir + self.exec_folder + "spi_pamp_input" + " " +
                      str(self.gnrl_cnt2)
                      )

    def deassertControlSignal(self, cnt_in):
        self.gnrl_cnt = self.gnrl_cnt & (~cnt_in)

        if self.PCBVer == 'v4.0_and_below':
            os.system(
                self.work_dir + self.exec_folder + "i2c_gnrl" + " " +
                str(self.gnrl_cnt)
            )
        elif self.PCBVer == 'v5.0':
            self.gnrl_cnt0 = self.gnrl_cnt & 0xffff
            self.gnrl_cnt1 = (self.gnrl_cnt >> 16) & 0xffff
            self.gnrl_cnt2 = (self.gnrl_cnt >> self.spi_pamp_U32) & 0xff
            os.system(
                self.work_dir + self.exec_folder + "i2c_gnrl" + " " +
                str(self.gnrl_cnt0) + " " +
                str(self.gnrl_cnt1)
            )
            os.system(self.work_dir + self.exec_folder + "spi_pamp_input" + " " +
                      str(self.gnrl_cnt2)
                      )

    def doLaplaceInversion(self, filename, outpath):
        # laplace inversion computatation
        os.system(
            self.work_dir + self.exec_folder + "nmr_sig_proc" + " " +
            filename + " " +
            outpath
        )
    
    def igbtPulse (self, plength, pspac, iter, sen_channel):
        os.system(
            self.work_dir + self.exec_folder + "igbt_pulser" + " " +
            str(plength[0]) + " " +
            str(plength[1]) + " " +
            str(plength[2]) + " " +
            str(plength[3]) + " " +
            str(plength[4]) + " " +
            str(plength[5]) + " " +
            str(plength[6]) + " " +
            str(plength[7]) + " " +
            str(plength[8]) + " " +
            str(plength[9]) + " " +
            str(plength[10]) + " " +
            str(plength[11]) + " " +
            str(plength[12]) + " " +
            str(plength[13]) + " " +
            str(plength[14]) + " " +
            str(plength[15]) + " " +  
            str(plength[16]) + " " +
            str(plength[17]) + " " +
            str(pspac) + " " +                      
            str(iter) + " " + 
            str(sen_channel)     
        )
        
    def igbtPulseMagnetControl (self, plength, pspac, iter):
        os.system(
            self.work_dir + self.exec_folder + "igbt_pulser_for_magnet_control" + " " +
            str(plength[0]) + " " +
            str(plength[1]) + " " +
            str(plength[2]) + " " +
            str(plength[3]) + " " +
            str(plength[4]) + " " +
            str(plength[5]) + " " +
            str(plength[6]) + " " +
            str(plength[7]) + " " +
            str(plength[8]) + " " +
            str(plength[9]) + " " +
            str(plength[10]) + " " +
            str(plength[11]) + " " +
            str(plength[12]) + " " +
            str(plength[13]) + " " +
            str(plength[14]) + " " +
            str(plength[15]) + " " +  
            str(plength[16]) + " " +
            str(plength[17]) + " " +
            str(pspac) + " " +                      
            str(iter)  
        )
        
    def igbtSenReading (self, sen_channel, num_iter):
        os.system(
            self.work_dir + self.exec_folder + "hall_reading" + " " +
            str(sen_channel) + " " +
            str(num_iter) 
        )
    
    
    def cpmgSequence(self, cpmg_freq, pulse1_us, pulse2_us, pulse1_dtcl, pulse2_dtcl, echo_spacing_us, scan_spacing_us, samples_per_echo, echoes_per_scan, init_adc_delay_compensation, number_of_iteration, ph_cycl_en, pulse180_t1_int, delay180_t1_int):
        # execute cpmg sequence
        command = (self.work_dir + self.exec_folder + "cpmg_iterate" + " " +
                   str(cpmg_freq) + " " +
                   str(pulse1_us) + " " +
                   str(pulse2_us) + " " +
                   str(pulse1_dtcl) + " " +
                   str(pulse2_dtcl) + " " +
                   str(echo_spacing_us) + " " +
                   str(scan_spacing_us) + " " +
                   str(samples_per_echo) + " " +
                   str(echoes_per_scan) + " " +
                   str(init_adc_delay_compensation) + " " +
                   str(number_of_iteration) + " " +
                   str(ph_cycl_en) + " " +
                   str(pulse180_t1_int) + " " +
                   str(delay180_t1_int)
                   )
        os.system(command)  # execute command & ignore its console

    def cpmgSequenceDirectRead(self, cpmg_freq, pulse1_us, pulse2_us, pulse1_dtcl, pulse2_dtcl, echo_spacing_us, scan_spacing_us, samples_per_echo, echoes_per_scan, init_adc_delay_compensation, number_of_iteration, ph_cycl_en, pulse180_t1_int, delay180_t1_int):
        data = np.zeros(samples_per_echo * echoes_per_scan)

        for i in range(0, number_of_iteration):
            # execute cpmg sequence
            command = (self.work_dir + self.exec_folder + "cpmg_iterate_direct" + " " +
                       str(cpmg_freq) + " " +
                       str(pulse1_us) + " " +
                       str(pulse2_us) + " " +
                       str(pulse1_dtcl) + " " +
                       str(pulse2_dtcl) + " " +
                       str(echo_spacing_us) + " " +
                       str(scan_spacing_us) + " " +
                       str(samples_per_echo) + " " +
                       str(echoes_per_scan) + " " +
                       str(init_adc_delay_compensation) + " " +
                       str(1) + " " +
                       str(ph_cycl_en) + " " +
                       str(pulse180_t1_int) + " " +
                       str(delay180_t1_int)
                       )
            os.system(command)  # execute command & ignore its console

            # read the data from SDRAM
            fpgaObj = fpga_de1soc()
            # get data averaged by number of iteration
            one_scan = fpgaObj.readSDRAM(samples_per_echo *
                                         echoes_per_scan)
            if (ph_cycl_en):
                if (i % 2):  # phase cycling every other scan
                    data = data - np.divide(one_scan, number_of_iteration)
                else:
                    data = data + np.divide(one_scan, number_of_iteration)
            else:
                data = data + np.divide(one_scan, number_of_iteration)

        return data

    def fid(self, cpmg_freq, pulse2_us, pulse2_dtcl, scan_spacing_us, samples_per_echo, number_of_iteration):
        # execute cpmg sequence
        command = (self.work_dir + self.exec_folder + "fid" + " " +
                   str(cpmg_freq) + " " +
                   str(pulse2_us) + " " +
                   str(pulse2_dtcl) + " " +
                   str(scan_spacing_us) + " " +
                   str(samples_per_echo) + " " +
                   str(number_of_iteration)
                   )
        os.system(command)  # execute command & ignore its console

    def noise(self, samp_freq, samples):
        scan_spacing_us = 100000
        number_of_iteration = 1
        # execute noise sequence
        command = (self.work_dir + self.exec_folder + "noise" + " " +
                   str(samp_freq) + " " +
                   str(scan_spacing_us) + " " +
                   str(samples) + " " +
                   str(number_of_iteration)
                   )
        os.system(command)  # execute command & ignore its console

    def wobble(self, sta_freq, sto_freq, spac_freq, samp_freq):
        # execute cpmg sequence
        command = (self.work_dir + self.exec_folder + "wobble" + " " +
                   str(sta_freq) + " " +
                   str(sto_freq) + " " +
                   str(spac_freq) + " " +
                   str(samp_freq)
                   )
        os.system(command)  # execute command & ignore its console

    def cpmgT1(self, cpmg_freq, pulse1_us, pulse2_us, pulse1_dtcl, pulse2_dtcl, echo_spacing_us, scan_spacing_us, samples_per_echo, echoes_per_scan, init_adc_delay_compensation, number_of_iteration, ph_cycl_en, pulse180_t1_us, logsw, delay180_sta, delay180_sto, delay180_ste, ref_number_of_iteration, ref_twait_mult, data_folder, en_scan_fig, en_fig):

        # create t1 measurement folder
        t1_meas_folder = datetime.now().strftime('%Y_%m_%d_%H_%M_%S') + '_t1_meas'
        os.mkdir(t1_meas_folder)
        t1_meas_hist = 't1_meas_hist.txt'  # the history file name for t1 measurement

        self.fig_num = 1
        self.fcpmg_to_fsys_mult = 16  # system_frequency/cpmg_frequency,set by fpga
        self.t1_opt_mult = 1.6

        # compute period for the system clock (which is multiplication of the cpmg
        # freq)
        t_sys = (1 / cpmg_freq) / self.fcpmg_to_fsys_mult

        # compute pulse180_t1 in integer values and round it to
        # fcpmg_to_fsys_mult multiplication
        pulse180_t1_int = np.round(
            (pulse180_t1_us / t_sys) / self.fcpmg_to_fsys_mult) * self.fcpmg_to_fsys_mult

        # process delay
        if logsw:
            delay180_t1_sw = np.logspace(
                np.log10(delay180_sta), np.log10(delay180_sto), delay180_ste)
        else:
            delay180_t1_sw = np.linspace(
                delay180_sta, delay180_sto, delay180_ste)
        # make delay to be multiplication of fcpmg_to_fsys_mult
        delay180_t1_sw_int = np.round((delay180_t1_sw / t_sys) /
                                      self.fcpmg_to_fsys_mult) * self.fcpmg_to_fsys_mult

        # compute the reference and do cpmg
        ref_twait = ref_twait_mult * delay180_t1_sw_int[delay180_ste - 1]
        ref_twait_int = np.round(
            (ref_twait) / self.fcpmg_to_fsys_mult) * self.fcpmg_to_fsys_mult
        self.cpmgSequence(cpmg_freq, pulse1_us, pulse2_us, pulse1_dtcl, pulse2_dtcl, echo_spacing_us, scan_spacing_us, samples_per_echo,
                          echoes_per_scan, init_adc_delay_compensation, ref_number_of_iteration, ph_cycl_en, pulse180_t1_int, ref_twait_int)
        # process the data
        meas_folder = parse_simple_info(data_folder, 'current_folder.txt')
        (a_ref, _, a0_ref, snr_ref, T2_ref, noise_ref, res_ref, theta_ref, data_filt_ref, echo_avg_ref, Df, _) = compute_iterate(
            data_folder, meas_folder[0], 0, 0, 0, en_scan_fig)

        # move the folder to t1 measurement folder and write history
        shutil.move(meas_folder[0], t1_meas_folder)
        write_text_append(t1_meas_folder, t1_meas_hist, meas_folder[0])

        # make the loop
        a0_table = np.zeros(delay180_ste)  # normal format
        a0_table_decay = np.zeros(delay180_ste)  # decay format
        asum_table = np.zeros(delay180_ste)  # normal format
        asum_table_decay = np.zeros(delay180_ste)  # decay format
        for i in range(0, delay180_ste):
            delay180_t1_int = delay180_t1_sw_int[i]

            # do cpmg scan
            self.cpmgSequence(cpmg_freq, pulse1_us, pulse2_us, pulse1_dtcl, pulse2_dtcl, echo_spacing_us, scan_spacing_us, samples_per_echo,
                              echoes_per_scan, init_adc_delay_compensation, number_of_iteration, ph_cycl_en, pulse180_t1_int, delay180_t1_int)
            # process the data (note that a0 and T2 is based on single
            # exponential fit)
            meas_folder = parse_simple_info(data_folder, 'current_folder.txt')
            (a, _, a0, snr, T2, noise, res, theta, data_filt, echo_avg, Df, _) = compute_iterate(
                data_folder, meas_folder[0], 1, theta_ref, echo_avg_ref, en_scan_fig)

            # move the folder to t1 measurement folder and write history
            shutil.move(meas_folder[0], t1_meas_folder)
            write_text_append(t1_meas_folder, t1_meas_hist, meas_folder[0])

            # interscan data store
            a0_table[i] = a0
            a0_table_decay[i] = a0_ref - a0
            asum_table[i] = np.mean(np.real(a))
            asum_table_decay[i] = np.mean(np.real(a_ref)) - np.mean(np.real(a))

            if en_fig:
                print('Loading Figure')
                plt.ion()
                fig = plt.figure(self.fig_num)
                fig.clf()

                ax = fig.add_subplot(3, 1, 1)
                if logsw:
                    line1, = ax.semilogx(
                        delay180_t1_sw[0:i + 1] / 1000, asum_table[0:i + 1], 'r-')
                else:
                    line1, = ax.plot(
                        delay180_t1_sw[0:i + 1] / 1000, asum_table[0:i + 1], 'r-')

                # ax.set_xlim(-50, 0)
                # ax.set_ylim(-50, 0)
                ax.set_ylabel('Initial amplitude [a.u.]')
                ax.set_title("T1 inversion recovery")
                ax.grid()

                ax = fig.add_subplot(3, 1, 2)
                if logsw:
                    line1, = ax.semilogx(
                        delay180_t1_sw[0:i + 1] / 1000, asum_table_decay[0:i + 1], 'r-')
                else:
                    line1, = ax.plot(
                        delay180_t1_sw[0:i + 1] / 1000, asum_table_decay[0:i + 1], 'r-')
                # ax.set_xlim(-50, 0)
                # ax.set_ylim(-50, 0)
                # ax.set_xlabel('Wait time [ms]')
                ax.set_ylabel('Initial amplitude [a.u.]')
                ax.grid()

                ax = fig.add_subplot(3, 1, 3)
                ax.set_ylabel('Amplitude [a.u.]')
                ax.set_xlabel('Wait time [ms]')
                ax.grid()

                fig.canvas.draw()
                fig.canvas.flush_events()
                print('Figure Loaded')
        # save t1 data to csv file to be processed
        f = open(t1_meas_folder + '/' + 't1heel_in.csv', "w+")
        for i in range(0, delay180_ste):
            f.write("%f," % (delay180_t1_sw[i] / 1000))  # in milisecond
            f.write("%f\n" % (a0_table_decay[i]))
        f.close()

        # process t1 data
        self.doLaplaceInversion(t1_meas_folder + '/' + 't1heel_in.csv',
                                t1_meas_folder)
        tvect, data = parse_csv_float2col(
            t1_meas_folder, 't1heel_out.csv')

        i_peaks = signal.find_peaks_cwt(data, np.arange(1, 10))

        t1_opt = tvect[max(i_peaks)]
        '''
        a_peaks = np.zeros(len(i_peaks))
        for i in range(0, len(i_peaks)):
            a_peaks[i] = data[i_peaks[i]]

        # find tvect in which the largest peak is found
        t1_opt = tvect[i_peaks[np.where(max(a_peaks))[0][0]]]  # in second
        '''

        if en_fig:
            ax = fig.add_subplot(3, 1, 3)
            if logsw:
                line1, = ax.semilogx(np.multiply(tvect, 1000), data, 'r-')
            else:
                line1, = ax.plot(np.multiply(tvect, 1000), data, 'r-')
            ax.set_ylabel('Amplitude [a.u.]')
            ax.set_xlabel('Wait time [ms]')
            ax.grid()
            fig.canvas.draw()

        # copy the measurement history script
        shutil.copy('measurement_history_matlab_script.txt', t1_meas_folder)

        return delay180_t1_sw, a0_table, a0_ref, asum_table, t1_opt, t1_meas_folder