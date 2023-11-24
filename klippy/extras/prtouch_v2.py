# prtouch support
#
# Copyright (C) 2018-2021  Creality <wangyulong878@sina.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import time
import socket
from . import probe
import math
import random
import threading



# COMMANDS

# READ_PRES C=100
# TEST_SWAP
# DEAL_AVGS
# TRIG_TEST C=10
# SELF_CHECK_PRTOUCH
# START_STEP_PRTOUCH DIR=1 SPD=10 DIS=10
# NOZZLE_CLEAR HOT_MIN_TEMP=140 HOT_MAX_TEMP=200 BED_MAX_TEMP=60
# CHECK_BED_MESH AUTO_G29=0
# PRTOUCH_READY
# SET_OVR_DET HOLD=50000
# FORCE_MOVE STEPPER=stepper_x DISTANCE=5 VELOCITY=10
# PID_CALIBRATE HEATER=extruder TARGET=210
# PID_CALIBRATE HEATER=heater_bed TARGET=60
# SAFE_DOWN_Z DOWN_DIS=5 


PR_ERR_CODE_PRES_READ_DATA_TIMEOUT  = {'code':'key401', 'msg':'PR_ERR_CODE_PRES_READ_DATA_TIMEOUT: The data read interval is too large, need=11ms, actual={0}.', 'values':[]}
PR_ERR_CODE_PRES_VAL_IS_CONSTANT    = {'code':'key401', 'msg':'PR_ERR_CODE_PRES_VAL_IS_CONSTANT: The pressure data for channel={0} is incorrect. The value is constant {1}.', 'values':[]}
PR_ERR_CODE_PRES_NOT_BE_SENSED      = {'code':'key401', 'msg':'PR_ERR_CODE_PRES_NOT_BE_SENSED: The pressure data in channel={0} cannot be properly sensed.', 'values':[]}
PR_ERR_CODE_PRES_LOST_RUN_DATA      = {'code':'key401', 'msg':'PR_ERR_CODE_PRES_LOST_RUN_DATA: The pressure data is lost when the probe is over and waiting for the data to be sent back.', 'values':[]}
PR_ERR_CODE_PRES_NOISE_TOO_BIG      = {'code':'key401', 'msg':'PR_ERR_CODE_PRES_NOISE_TOO_BIG: Sensor data noise is too big, channel={0}.', 'values':[]}
PR_ERR_CODE_PRESS_TOO_LARGE         = {'code':'key401', 'msg':'PR_ERR_CODE_PRESS_TOO_LARGE: Excessive pressure was detected, possibly from the nozzle scraping the hot bed, channel={0}.', 'values':[]} 
PR_ERR_CODE_HAVE_LOST_STEP          = {'code':'key401', 'msg':'PR_ERR_CODE_HAVE_LOST_STEP: Z-axis motor step loss was found.', 'values':[]}
PR_ERR_CODE_STEP_LOST_RUN_DATA      = {'code':'key401', 'msg':'PR_ERR_CODE_STEP_LOST_RUN_DATA: The motor step data is lost when the probe is over and waiting for data return', 'values':[]}
PR_ERR_CODE_FATESTART_TO_LOW        = {'code':'key401', 'msg':'PR_ERR_CODE_FATESTART_TO_LOW: [bed_mesh] -> Fate_start is too small and must be greater than 0..', 'values':[]}
PR_ERR_CODE_G28_Z_DETECTION_TIMEOUT = {'code':'key401', 'msg':'PR_ERR_CODE_G28_Z_DETECTION_TIMEOUT: G28 Z probe timed out.', 'values':[]}
PR_ERR_CODE_G29_Z_DETECTION_TIMEOUT = {'code':'key401', 'msg':'PR_ERR_CODE_G29_Z_DETECTION_TIMEOUT: G29 Z probe timed out.', 'values':[]}
PR_ERR_CODE_SWAP_PIN_DETECTI        = {'code':'key401', 'msg':'PR_ERR_CODE_SWAP_PIN_DETECTI: The synchronization pin test failed, pres_swap_pin={0}, step_swap_pin={1}.', 'values':[]} 
PR_ERR_CODE_CK_BED_MESH_OUT_RANGE   = {'code':'key401', 'msg':'PR_ERR_CODE_CK_BED_MESH_OUT_RANGE: A hot bed tilting procedure that differs too much from bed_mesh data.', 'values':[]}
PR_ERR_CODE_CAL_DATA_ERROR          = {'code':'key401', 'msg':'PR_ERR_CODE_CAL_DATA_ERROR: An error occurred during data calculation..', 'values':[]}


MAX_PRES_CNT = 4
MAX_STEP_CNT = 4
MAX_BUF_LEN = 32


# PRTouch "endstop" wrapper
class PRTouchEndstopWrapper:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.steppers = []
        self.dbg_msg = {}
        self.shut_down = False
        self.jump_probe_ready = False
        self.mm_per_step, self.pres_tri_time, self.step_tri_time, self.pres_tri_chs = 0, 0, 0, 0
        self.rdy_pos, self.gap_pos = None, None
        self.ovr_det_hold, self.mm_per_step = 0, None
        self.sys_max_velocity, self.sys_max_accel, self.sys_max_z_velocity, self.sys_max_z_accel = 0, 0, 0, 0

        self.step_res, self.pres_res, self.steppers = [], [], []
        self.read_swap_prtouch_cmd, self.start_step_prtouch_cmd = None, None
        self.write_swap_prtouch_cmd, self.read_pres_prtouch_cmd, self.start_pres_prtouch_cmd = None, None, None
        self.bed_mesh, self.toolhead, self.bed_mesh, self.pheaters, self.heater_hot, self.heater_bed = None, None, None, None, None, None
        # 0. Base Cfg
        self.use_adc            = config.getboolean('use_adc', default=False)
        # 1. Tri Cfg
        self.tri_acq_ms         = config.getint('tri_acq_ms', default=(1 if self.use_adc else 12), minval=1, maxval=1000) 
        self.tri_send_ms        = config.getint('tri_send_ms', default=3, minval=1, maxval=1000) 
        self.tri_need_cnt       = config.getint('tri_need_cnt', default=1, minval=1, maxval=MAX_PRES_CNT) 
        self.tri_hftr_cut       = config.getfloat('tri_hftr_cut', default=2, minval=0.01, maxval=100.)
        self.tri_lftr_k1        = config.getfloat('tri_lftr_k1', default=(0.50 if self.use_adc else 0.95), minval=0., maxval=1.)
        self.tri_min_hold       = config.getint('tri_min_hold', default=(3 if self.use_adc else 2000), minval=1, maxval=200000)
        self.tri_max_hold       = config.getint('tri_max_hold', default=(3072 if self.use_adc else 6000), minval=self.tri_min_hold, maxval=600000)
        # 2. Debug Cfg
        self.show_msg           = config.getboolean('show_msg', default=False)
        self.tri_wave_ip        = config.get('tri_wave_ip', None)        
        # 3. Shake Z Cfg
        self.shake_cnt          = config.getint('shake_cnt', default=8, minval=1, maxval=512)
        self.shake_range        = config.getint('shake_range', default=0.5, minval=0.1, maxval=2)
        # 4. Clear Nozzle Cfg
        self.hot_min_temp       = config.getfloat('hot_min_temp', default=140, minval=80, maxval=200)
        self.hot_max_temp       = config.getfloat('hot_max_temp', default=200, minval=180, maxval=300)
        self.bed_max_temp       = config.getfloat('bed_max_temp', default=60, minval=45, maxval=100)
        self.pa_clr_dis_mm      = config.getfloat('pa_clr_dis_mm', default=30, minval=2, maxval=100)
        self.pa_clr_down_mm     = config.getfloat('pa_clr_down_mm', default=-0.15, minval=-1, maxval=1)
        self.clr_noz_start_x    = config.getfloat('clr_noz_start_x', default=0, minval=0, maxval=1000)
        self.clr_noz_start_y    = config.getfloat('clr_noz_start_y', default=0, minval=0, maxval=1000)
        self.clr_noz_len_x      = config.getfloat('clr_noz_len_x', default=0, minval=self.pa_clr_dis_mm + 6, maxval=1000)
        self.clr_noz_len_y      = config.getfloat('clr_noz_len_y', default=0, minval=0, maxval=1000)    
        self.clr_xy_spd         = config.getfloat('clr_xy_spd', default=2.0, minval=0.1, maxval=10) 
        # 5. Speed Cfg
        self.tri_z_down_spd     = config.getfloat('speed', default=(10 if self.use_adc else 2.5), minval=0.1, maxval=100) # speed
        self.tri_z_up_spd       = config.getfloat('lift_speed', self.tri_z_down_spd * (1.0 if self.use_adc else 2.0), minval=0.1, maxval=100)
        self.rdy_xy_spd         = config.getfloat('rdy_xy_spd', default=(250 if self.use_adc else 200), minval=1, maxval=1000)
        self.rdy_z_spd          = config.getfloat('rdy_z_spd', default=self.tri_z_up_spd, minval=1, maxval=50)
        self.acc_ctl_mm         = config.getfloat('acc_ctl_mm', (0.5 if self.use_adc else 0.25), minval=0, maxval=10)
        self.low_spd_nul        = config.getint('low_spd_nul', 5, minval=1, maxval=10)
        self.send_step_duty     = config.getint('send_step_duty', 16, minval=0, maxval=10)
        self.run_max_velocity   = config.getfloat('run_max_velocity', default=500, minval=1, maxval=5000)
        self.run_max_accel      = config.getfloat('run_max_accel', default=500, minval=1, maxval=50000)  
        self.run_max_z_velocity = config.getfloat('run_max_z_velocity', default=20, minval=1, maxval=5000)
        self.run_max_z_accel    = config.getfloat('run_max_z_accel', default=200, minval=1, maxval=50000) 
        # 6. Gap Cfg
        self.z_gap_spd          = config.getfloat('z_gap_spd', default=(2 if self.use_adc else 0.6), minval=0, maxval=1)
        self.need_measure_gap   = config.getboolean('need_measure_gap', default=False) #  not self.use_adc
        self.gap_dis_range      = config.getfloat('gap_dis_range', default=0.6, minval=0.2, maxval=2)
        self.check_bed_mesh_max_err = config.getfloat('check_bed_mesh_max_err', default=0.2, minval=0.01, maxval=1)
        self.stored_profs       = config.get_prefix_sections('prtouch')
        self.stored_profs       = self.stored_profs[1] if (len(self.stored_profs) == 2 and self.need_measure_gap) else None
        # 6. Other Cfg
        self.need_self_check    = config.getboolean('need_self_check', default=False) 
        self.bed_max_err        = config.getfloat('bed_max_err', (5 if self.use_adc else 2.5), minval=1, maxval=10)  
        self.max_z              = config.getsection('stepper_z').getfloat('position_max', default=300, minval=10, maxval=500)        
        self.g29_down_min_z     = config.getfloat('g29_down_min_z', 300, 5, 500)
        self.probe_min_3err     = config.getfloat('probe_min_3err', 0.1, 0.01, 1)
        self.step_base          = config.getint('step_base', 1, minval=1, maxval=10) 
        self.g28_wait_cool_down = config.getboolean('g28_wait_cool_down', default=False)
        self.best_above_z       = config.getfloat('best_above_z', default=(2 if self.use_adc else 1.5), minval=0.5, maxval=10)      # above 
        # 7. Fan Cfg
        self.fan_heat_min_spd   = config.getfloat('fan_heat_min_spd', default=0.3, minval=0, maxval=255) 
        self.fan_heat_max_spd   = config.getfloat('fan_heat_max_spd', default=1.0, minval=0, maxval=255)

        self.gcode              = self.printer.lookup_object('gcode')
        self.lost_step_dis      = config.getfloat('lost_step_dis', default=0.5, minval=0, maxval=10)
        self.tilt_corr_dis      = config.getfloat('tilt_corr_dis', default=-1, minval=-1, maxval=0.1)
        self.noz_ex_com         = config.getfloat('noz_ex_com', default=0.0, minval=0, maxval=1)    # Nozzle expansion compensation
        self.sys_time_duty      = config.getfloat('sys_time_duty', default=0.001, minval=0.00001, maxval=0.010)
        self.is_corexz          = str(config.getsection('printer').get('kinematics')) == 'corexz'
        # 1. Load Swap Pins
        self.step_swap_pin = config.get('step_swap_pin')
        self.pres_swap_pin = config.get('pres_swap_pin')
        # 1. Load Pres Pins
        self.pres_cnt = config.getint('pres_cnt', 1, 1, MAX_PRES_CNT) 
        self.pres_clk_pins, self.pres_sdo_pins, self.pres_adc_pins = [], [], []
        for i in range(self.pres_cnt):
            if self.use_adc:
                self.pres_adc_pins.append(config.get('pres%d_adc_pins' % i))
            else:
                self.pres_clk_pins.append(config.get('pres%d_clk_pins' % i))
                self.pres_sdo_pins.append(config.get('pres%d_sdo_pins' % i))
        # 2. Load Step Pins
        self.z_step_pins, self.z_dir_pins = [], []
        for s in ['stepper_z', 'stepper_x' if self.is_corexz else 'stepper_z1', 'stepper_z2', 'stepper_z3']:
            if config.has_section(s):
                sec = config.getsection(s)
                self.z_step_pins.append(sec.get('step_pin'))
                self.z_dir_pins.append(sec.get('dir_pin'))
            pass
        # 3. Creat Step And Pres Oid
        self.ppins = self.printer.lookup_object('pins')    
        self.step_mcu, self.pres_mcu = self.ppins.chips['mcu'], self.ppins.chips['mcu']
        if self.step_swap_pin and self.pres_swap_pin:
            self.step_mcu = self.ppins.parse_pin(self.step_swap_pin, True, True)['chip']
            self.pres_mcu = self.ppins.parse_pin(self.pres_swap_pin, True, True)['chip']
        self.step_oid, self.pres_oid = self.step_mcu.create_oid(), self.pres_mcu.create_oid()

        self.step_mcu.register_config_callback(self._build_step_config)
        self.pres_mcu.register_config_callback(self._build_pres_config)


        self.gcode.register_command('TEST_PRTH', self.cmd_TEST_PRTH, desc=self.cmd_TEST_PRTH_help)
        self.gcode.register_command('READ_PRES', self.cmd_READ_PRES, desc=self.cmd_READ_PRES_help)
        self.gcode.register_command('TEST_SWAP', self.cmd_TEST_SWAP, desc=self.cmd_TEST_SWAP_help)
        self.gcode.register_command('DEAL_AVGS', self.cmd_DEAL_AVGS, desc=self.cmd_DEAL_AVGS_help)
        self.gcode.register_command('TRIG_TEST', self.cmd_TRIG_TEST, desc=self.cmd_TRIG_TEST_help)
        self.gcode.register_command('SET_OVR_DET', self.cmd_SET_OVR_DET, desc=self.cmd_SET_OVR_DET_help)        
        self.gcode.register_command('CHECK_BED_MESH', self.cmd_CHECK_BED_MESH, desc=self.cmd_CHECK_BED_MESH_help)
        self.gcode.register_command('PRTOUCH_READY', self.cmd_PRTOUCH_READY, desc=self.cmd_PRTOUCH_READY_help)     
        self.gcode.register_command('NOZZLE_CLEAR', self.cmd_NOZZLE_CLEAR, desc=self.cmd_NOZZLE_CLEAR_help)       
        self.gcode.register_command('SAFE_DOWN_Z', self.cmd_SAFE_DOWN_Z, desc=self.cmd_SAFE_DOWN_Z_help)     
        self.gcode.register_command('SELF_CHECK_PRTOUCH', self.cmd_SELF_CHECK_PRTOUCH, desc=self.cmd_SELF_CHECK_PRTOUCH_help)
        self.gcode.register_command('START_STEP_PRTOUCH', self.cmd_START_STEP_PRTOUCH, desc=self.cmd_START_STEP_PRTOUCH_help)


        self.step_mcu.register_response(self._handle_step_debug_prtouch, "debug_prtouch", self.step_oid)
        self.step_mcu.register_response(self._handle_result_run_step_prtouch, "result_run_step_prtouch", self.step_oid)
        
        self.pres_mcu.register_response(self._handle_pres_debug_prtouch, "debug_prtouch", self.pres_oid)
        self.pres_mcu.register_response(self._handle_result_run_pres_prtouch, "result_run_pres_prtouch", self.pres_oid)
        self.pres_mcu.register_response(self._handle_result_read_pres_prtouch, "result_read_pres_prtouch", self.pres_oid)
        pass

    def _build_step_config(self):
        self.bed_mesh   = self.printer.lookup_object('bed_mesh')
        self.toolhead   = self.printer.lookup_object("toolhead")
        self.probe      = self.printer.lookup_object("probe")
        self.pheaters   = self.printer.lookup_object('heaters')
        self.heater_hot = self.printer.lookup_object('extruder').heater
        self.heater_bed = self.printer.lookup_object('heater_bed').heater

        if 'heater_fan' in self.printer.objects:
            self.fan_heat_obj = self.printer.lookup_object('heater_fan')
        if 'fan' in self.printer.objects:
            self.fan_mode_obj = self.printer.lookup_object('fan')

        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max

        if self.tilt_corr_dis == -1:
            self.tilt_corr_dis = 0.02 if max_x < 250 else 0.04

        if self.clr_noz_start_x <= 0 or self.clr_noz_start_y <= 0 or self.clr_noz_len_x <= 0 or self.clr_noz_len_y <= 0:
            self.clr_noz_start_x = (max_x - min_x) * 1 / 3 + min_x
            self.clr_noz_start_y = max_y - 6
            self.clr_noz_len_x = (max_x - min_x) * 1 / 3
            self.clr_noz_len_y = 5

        random.seed(time.time()) 
        self.rdy_pos = [[min_x - 2, min_y - 2, self.bed_max_err], [min_x - 2, max_y + 2, self.bed_max_err],
                        [max_x + 2, max_y + 2, self.bed_max_err], [max_x + 2, min_y - 2, self.bed_max_err]]
        self.gap_pos = [[min_x - 2, min_y - 2, 0 if self.stored_profs is None else self.stored_profs.getfloat('z_gap_00', default=0, minval=0, maxval=1)],
                        [min_x - 2, max_y + 2, 0 if self.stored_profs is None else self.stored_profs.getfloat('z_gap_01', default=0, minval=0, maxval=1)],
                        [max_x + 2, max_y + 2, 0 if self.stored_profs is None else self.stored_profs.getfloat('z_gap_11', default=0, minval=0, maxval=1)],
                        [max_x + 2, min_y - 2, 0 if self.stored_profs is None else self.stored_profs.getfloat('z_gap_10', default=0, minval=0, maxval=1)]]
        
        self.step_mcu.add_config_cmd('config_step_prtouch oid=%d step_cnt=%d swap_pin=%s sys_time_duty=%u' 
                                     % (self.step_oid, len(self.z_step_pins), self.ppins.parse_pin(self.step_swap_pin, True, True)['pin'], int(self.sys_time_duty * 100000)))

        for i in range(len(self.z_step_pins)):
            step_par = self.ppins.parse_pin(self.z_step_pins[i], True, True)
            dir_par = self.ppins.parse_pin(self.z_dir_pins[i], True, True)
            if self.is_corexz:
                if i == 0:
                    self.step_mcu.add_config_cmd('add_step_prtouch oid=%d index=%d dir_pin=%s step_pin=%s dir_invert=%d step_invert=%d' % (self.step_oid, i, dir_par['pin'], step_par['pin'], not dir_par['invert'], step_par['invert']))
                else:
                    self.step_mcu.add_config_cmd('add_step_prtouch oid=%d index=%d dir_pin=%s step_pin=%s dir_invert=%d step_invert=%d' % (self.step_oid, i, dir_par['pin'], step_par['pin'], dir_par['invert'], step_par['invert']))
            else:
                self.step_mcu.add_config_cmd('add_step_prtouch oid=%d index=%d dir_pin=%s step_pin=%s dir_invert=%d step_invert=%d' % (self.step_oid, i, dir_par['pin'], step_par['pin'], dir_par['invert'], step_par['invert']))
            
        self.read_swap_prtouch_cmd = self.step_mcu.lookup_query_command('read_swap_prtouch oid=%c', 'result_read_swap_prtouch oid=%c sta=%c', oid=self.step_oid)
        self.start_step_prtouch_cmd = self.step_mcu.lookup_command('start_step_prtouch oid=%c dir=%c send_ms=%c step_cnt=%u step_us=%u acc_ctl_cnt=%u low_spd_nul=%c send_step_duty=%c auto_rtn=%c', cq=None)
        pass

    def _build_pres_config(self):
        self.pres_mcu.add_config_cmd('config_pres_prtouch oid=%d use_adc=%d pres_cnt=%d swap_pin=%s sys_time_duty=%u' 
                                     % (self.pres_oid, self.use_adc, self.pres_cnt, self.ppins.parse_pin(self.pres_swap_pin, True, True)['pin'], int(self.sys_time_duty * 100000)))
        for i in range(self.pres_cnt):
            if self.use_adc:
                adc_par = self.ppins.parse_pin(self.pres_adc_pins[i], True, True)
                self.pres_mcu.add_config_cmd('add_pres_prtouch oid=%d index=%d clk_pin=%s sda_pin=%s' % (self.pres_oid, i, adc_par['pin'], adc_par['pin']))
            else:
                clk_par = self.ppins.parse_pin(self.pres_clk_pins[i], True, True)
                sdo_par = self.ppins.parse_pin(self.pres_sdo_pins[i], True, True)
                self.pres_mcu.add_config_cmd('add_pres_prtouch oid=%d index=%d clk_pin=%s sda_pin=%s' % (self.pres_oid, i, clk_par['pin'], sdo_par['pin']))
   
        self.write_swap_prtouch_cmd = self.pres_mcu.lookup_query_command('write_swap_prtouch oid=%c sta=%c', 'resault_write_swap_prtouch oid=%c', oid=self.pres_oid)
        self.read_pres_prtouch_cmd = self.pres_mcu.lookup_command('read_pres_prtouch oid=%c acq_ms=%u cnt=%u', cq=None)
        self.start_pres_prtouch_cmd = self.pres_mcu.lookup_command('start_pres_prtouch oid=%c tri_dir=%c acq_ms=%c send_ms=%c need_cnt=%c hftr_cut=%u lftr_k1=%u min_hold=%u max_hold=%u', cq=None)
        self.deal_avgs_prtouch_cmd = self.pres_mcu.lookup_query_command('deal_avgs_prtouch oid=%c base_cnt=%c', 'result_deal_avgs_prtouch oid=%c ch0=%i ch1=%i ch2=%i ch3=%i', oid=self.pres_oid)
        pass
    
    #region Handles
    def _handle_step_debug_prtouch(self, params):
        # self.print_msg('HD_STEP_DEBUG', str(params))
        pass

    def _handle_result_run_step_prtouch(self, params):
        # self.print_msg('HD_RUN_STEP', str(params))
        self.step_tri_time = params['tri_time'] / 10000.
        for i in range(4):
            sdir = {'tick': params['tick%d' % i] / 10000., 'step': params['step%d' % i]}
            self.step_res.append(sdir)
        pass
    
    def _handle_pres_debug_prtouch(self, params):
        # self.print_msg('HD_PRES_DEBUG', str(params))
        pass

    def _handle_result_run_pres_prtouch(self, params):
        # self.print_msg('HD_RUN_PRES', str(params))
        self.pres_tri_time = params['tri_time'] / 10000.
        self.pres_tri_chs = params['tri_chs']
        for i in range(2):
            rdir = {'tick':params['tick_%d' % i] / 10000., 'ch0':params['ch0_%d' % i], 'ch1':params['ch1_%d' % i], 'ch2':params['ch2_%d' % i], 'ch3':params['ch3_%d' % i]}
            self.pres_res.append(rdir)
        pass 

    def _handle_result_read_pres_prtouch(self, params):
        # self.print_msg('HD_READ_PRES', str(params), True)
        self.pres_res.append(params)
        pass
    #endregion

    #region System
    def get_mcu(self):
        return self.step_mcu
    
    def get_position_endstop(self):
        return 0
    
    def add_stepper(self, stepper):
        self.mm_per_step = self.steppers[0].get_step_dist() * self.step_base
        if stepper in self.steppers:
            return
        self.steppers.append(stepper)

    def get_steppers(self):
        return list(self.steppers)

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        return True

    def home_wait(self, home_end_time):
        return True
    
    def multi_probe_begin(self):
        pass

    def multi_probe_end(self):
        pass

    def query_endstop(self, print_time):
        return False
    
    def probe_prepare(self, hmove):
        pass

    def probe_finish(self, hmove):
        pass
    #endregion

    #region Prtouch
    def print_msg(self, title, msg, force=False):
        if not self.show_msg and not force:
            return
        if title != 'SHOW_WAVE':
            self.gcode.respond_info('[' + title + ']' + msg)
        if self.tri_wave_ip is None:
            return
        if title not in self.dbg_msg:   
            self.dbg_msg[title] = len(self.dbg_msg)
        ss = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ss.sendto((str(self.dbg_msg[title]) + '$' + title + '$' + time.strftime("%H:%M:%S ")  + '$' +  msg + '\n').encode(), (str(self.tri_wave_ip), 21021))
        ss.close() 
        pass

    def print_ary(self, title, ary, lent=32, pt_cnt=2, force=False):
        if self.show_msg or force:
            st = '['
            for i in range(len(ary) - lent, len(ary)):
                st = st + ("%." + ("%df, " % pt_cnt)) % (ary[i])
            self.print_msg(title, st + ']', force)
        pass

    def print_res(self, title='None'):
        t_buf, p_buf = [], []
        for i in range(len(self.step_res)):
            t_buf.append(self.step_res[i]['tick'])
            p_buf.append(self.step_res[i]['step'])   
        self.print_ary('STEP_TICK', t_buf, len(t_buf))
        self.print_ary('STEP_DATA', p_buf, len(p_buf))

        t_buf, p_buf = [], [[], [], [], []]
        for i in range(len(self.pres_res)):
            t_buf.append(self.pres_res[i]['tick'])
            for j in range(self.pres_cnt):
                p_buf[j].append(self.pres_res[i]['ch%d' % j])

        self.print_ary('PRES_TICK', t_buf, len(t_buf))
        for i in range(self.pres_cnt):
            self.send_wave_tri(i, p_buf[i])
            self.print_ary('PRES_CH%d' % i, p_buf[i], len(p_buf[i]))
            pass
        pass

    def ck_and_raise_error(self, ck, err_code, vals=[]):  
        if not ck:
            return
        err_code['values'] = vals
        raise self.printer.command_error(str(err_code))

    def send_wave_tri(self, ch, ary):
        msg = '%d' % ch
        for i in range(len(ary)):
            msg += ',%d' % ary[i]
        self.print_msg('SHOW_WAVE', msg)
        pass

    def delay_s(self, delay_s):
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        if not self.printer.is_shutdown():
            self.toolhead.get_last_move_time()
            eventtime = reactor.pause(eventtime + delay_s)
            pass

    def get_linear2(self, p1, p2, po, is_base_x):
        if (math.fabs(p1[0] - p2[0]) < 0.001 and is_base_x) or (math.fabs(p1[1] - p2[1]) < 0.001 and not is_base_x):
            return po
        a = (p2[2] - p1[2]) / ((p2[0] - p1[0]) if is_base_x else (p2[1] - p1[1]))
        b = p1[2] - (p1[0] if is_base_x else p1[1]) * a
        po[2] = a * (po[0] if is_base_x else po[1]) + b
        return po

    def ck_g28ed(self):
        for i in range(3):
            if self.toolhead.kin.limits[i][0] > self.toolhead.kin.limits[i][1]:
                self.gcode.run_script_from_command('G28')
                break
        pass

    def set_step_par(self, load_sys=True):
        if load_sys:
            self.toolhead.max_velocity = self.sys_max_velocity
            self.toolhead.max_accel = self.sys_max_accel
            self.toolhead.kin.max_z_velocity = self.sys_max_z_velocity
            self.toolhead.kin.max_z_accel = self.sys_max_z_accel
            return

        self.sys_max_velocity = self.toolhead.max_velocity
        self.sys_max_accel = self.toolhead.max_accel
        self.sys_max_z_velocity = self.toolhead.kin.max_z_velocity
        self.sys_max_z_accel = self.toolhead.kin.max_z_accel

        self.toolhead.max_velocity = self.run_max_velocity
        self.toolhead.max_accel = self.run_max_accel
        self.toolhead.kin.max_z_velocity = self.run_max_z_velocity
        self.toolhead.kin.max_z_accel = self.run_max_z_accel
        pass    

    def enable_steps(self):
        for stepper in self.toolhead.get_kinematics().get_steppers():
            print_time = self.toolhead.get_last_move_time()
            stepper_enable = self.printer.lookup_object('stepper_enable')
            enable = stepper_enable.lookup_enable(stepper.get_name())
            was_enable = enable.is_motor_enabled()
            if not was_enable:
                enable.motor_enable(print_time)
                self.toolhead.dwell(0.100)
                self.delay_s(0.5) 
        pass 

    def move(self, pos, speed, wait=True):
        if not self.shut_down:
            self.gcode.run_script_from_command('G1 F%d X%.3f Y%.3f Z%.3f' % (speed * 60, pos[0], pos[1], pos[2]) if len(pos) >= 3 else 'G1 F%d X%.3f Y%.3f' % (speed * 60, pos[0], pos[1]))
            if wait:
                self.toolhead.wait_moves()
            pass
    #endregion

    def set_fan_speed(self, fan_name='None', fan_spd=0.):
        if fan_name == 'heater_fan':
            for key in self.printer.objects:
                if 'heater_fan' in key:
                    self.printer.objects[key].fan.set_speed_from_command(fan_spd)
                    break

        if fan_name == 'fan':
            if 'fan' in self.printer.objects:
                self.printer.lookup_object('fan').fan.set_speed_from_command(fan_spd)
            else:
                self.gcode.run_script_from_command('M106 P0 S%d' % (fan_spd * 255))
        pass

    def set_hot_temps(self, temp, wait=False, err=5):
        self.pheaters.set_temperature(self.heater_hot, temp, False)
        if wait:
            while not self.shut_down and abs(self.heater_hot.target_temp - self.heater_hot.smoothed_temp) > err and self.heater_hot.target_temp > 0:
                self.delay_s(0.100) 
        pass

    def set_bed_temps(self, temp, wait=False, err=5):
        self.pheaters.set_temperature(self.heater_bed, temp, False)           
        if wait:
            while not self.shut_down and abs(self.heater_bed.target_temp - self.heater_bed.smoothed_temp) > err and self.heater_bed.target_temp > 0:
                self.delay_s(0.100)          
        pass

    def shake_motor(self, cnt):
        now_pos = self.toolhead.get_position()
        max_z_velocity = self.toolhead.kin.max_z_velocity
        for i in range(int(cnt / 2)):
            self.gcode.run_script_from_command('G1 X%.2f Y%.2f Z%.2f F%d' % (now_pos[0] - self.shake_range, now_pos[1] - self.shake_range, now_pos[2] - self.shake_range / 2, int(60 * max_z_velocity * 0.8)))
            self.gcode.run_script_from_command('G1 X%.2f Y%.2f Z%.2f F%d' % (now_pos[0] + self.shake_range, now_pos[1] - self.shake_range, now_pos[2] + self.shake_range / 2, int(60 * max_z_velocity * 0.8)))
            self.gcode.run_script_from_command('G1 X%.2f Y%.2f Z%.2f F%d' % (now_pos[0] + self.shake_range, now_pos[1] + self.shake_range, now_pos[2] - self.shake_range / 2, int(60 * max_z_velocity * 0.8)))
            self.gcode.run_script_from_command('G1 X%.2f Y%.2f Z%.2f F%d' % (now_pos[0] - self.shake_range, now_pos[1] + self.shake_range, now_pos[2] + self.shake_range / 2, int(60 * max_z_velocity * 0.8)))
            while len(self.toolhead.move_queue.queue) >= 5:
                self.delay_s(0.010)
        self.move(now_pos, self.rdy_z_spd)
        pass

    def get_valid_ch(self):
        now_pos = self.toolhead.get_position()
        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max  
        l_chs = []
        l_chs.append(math.sqrt((now_pos[0] - min_x) ** 2 + (now_pos[1] - min_y) ** 2) if (self.pres_tri_chs & 0x01) else max_x * max_y * 2)
        l_chs.append(math.sqrt((now_pos[0] - max_x) ** 2 + (now_pos[1] - min_y) ** 2) if (self.pres_tri_chs & 0x02) else max_x * max_y * 2)
        l_chs.append(math.sqrt((now_pos[0] - min_x) ** 2 + (now_pos[1] - max_y) ** 2) if (self.pres_tri_chs & 0x04) else max_x * max_y * 2)
        l_chs.append(math.sqrt((now_pos[0] - max_y) ** 2 + (now_pos[1] - max_y) ** 2) if (self.pres_tri_chs & 0x08) else max_x * max_y * 2)
        valid_ch = l_chs.index(min(l_chs))         
        self.print_msg('VALID_CH', 'Tri_mark=%d best_ch=%d Chs=' % (self.pres_tri_chs, valid_ch) + '  ' + str(l_chs))
        return valid_ch, l_chs       

    def cal_tri_data(self, start_step, start_pos_z, step_res, pres_res, oft_z=0):   
        # 0. Send Debut Msg.
        for i in range(self.pres_cnt):
            tmp_buf = []
            for j in range(len(pres_res)):
                tmp_buf.append(pres_res[j]['ch%d' % i])
            self.send_wave_tri(i, tmp_buf)
            pass
        # 1. Get Best Tri Ch
        max_x, max_y = self.bed_mesh.bmc.mesh_max
        _valid_ch, l_chs = self.get_valid_ch()
        out_mms = []
        for valid_ch in range(len(l_chs)):
            if l_chs[valid_ch] == max_x * max_y * 2:
                continue
            # 2. Copy Pres Tick And Data.
            pres_t_buf, pres_d_buf = [], []
            for i in range(len(pres_res)):
                pres_t_buf.append(pres_res[i]['tick'] - self.pres_tri_time)       
                pres_d_buf.append(pres_res[i]['ch%d' % valid_ch])
                pass
            # self.print_ary('CAL_TRI_TICK', 'pres_t_buf=', pres_t_buf, len(pres_t_buf), 3)
            # self.print_ary('CAL_TRI_DATA', 'pres_d_buf=', pres_d_buf, len(pres_d_buf), 0)
            # 3. Copy Step Tick And Data.
            step_t_buf, step_d_buf = [], []
            for i in range(len(step_res)):
                step_t_buf.append(step_res[i]['tick'] - self.step_tri_time)
                step_d_buf.append(step_res[i]['step'])
                pass
            # self.print_ary('CAL_TRI_TICK', 'step_t_buf=', step_t_buf, len(step_t_buf), 3)
            # self.print_ary('CAL_TRI_DATA', 'step_d_buf=', step_d_buf, len(step_d_buf), 0)
            # 4. Cal The Pres Tri Index And Tick
            vals_p = [x for x in pres_d_buf]
            min_val, max_val = min(vals_p), max(vals_p)
            for i in range(len(vals_p)):
                vals_p[i] = (vals_p[i] - min_val) / (max_val - min_val)
            angle = math.atan((vals_p[-1] - vals_p[0]) / len(vals_p))
            sin_angle, cos_angle = math.sin(-angle), math.cos(-angle)
            for i in range(len(vals_p)):
                vals_p[i] = (i - 0) * sin_angle + (vals_p[i] - 0) * cos_angle + 0
            # self.print_ary('SINCOS', 'sincos = ', vals_p, len(vals_p), 2)
            pres_tri_index = vals_p.index(min(vals_p))
            pres_tri_tick = pres_t_buf[pres_tri_index]
            # 5. Cal The Step Tri Index And Tick
            step_tri_index = MAX_BUF_LEN - 1
            step_tri_tick = step_t_buf[step_tri_index]
            for i in range(len(step_t_buf) - 1):
                if ((step_t_buf[i] <= pres_tri_tick <= step_t_buf[i + 1]) or (step_t_buf[i] == pres_tri_tick)) and step_d_buf[i] != 0:
                    step_tri_index = i
                    step_tri_tick = step_t_buf[step_tri_index]
                    break
            out_step = step_d_buf[-1]
            if 0 < step_tri_index < len(step_res) - 1:
                out_step = step_d_buf[step_tri_index] + (step_d_buf[step_tri_index + 1] - step_d_buf[step_tri_index]) * (pres_tri_tick - step_tri_tick) / (step_t_buf[step_tri_index + 1] - step_t_buf[step_tri_index])
            out_val_mm = start_pos_z - (start_step - out_step) * self.mm_per_step + oft_z
            self.print_msg('TRI_PRES_MSG', 'start_pres=%d, tri_pres=%d, end_pres=%d, pres_tri_index=%d, pres_tri_tick=%.3f' % (pres_d_buf[0], pres_d_buf[pres_tri_index], pres_d_buf[-1], pres_tri_index, pres_tri_tick))
            self.print_msg('TRI_STEP_MSG', 'start_step=%d, tri_step=%d, end_step=%d, step_tri_index=%d, step_tri_tick=%.3f' % (start_step,    out_step,                   step_d_buf[-1], step_tri_index, step_tri_tick))
            self.print_msg('TRI_OUT_MM', 'mm_pre_step=%f, out_mm=%f' % (self.mm_per_step, out_val_mm))
            out_mms.append(out_val_mm)
        self.print_ary('OUT_MMS', out_mms, len(out_mms), 2)
        return sum(out_mms) / len(out_mms) # out_val_mm

    def get_step_cnts(self, run_dis, run_spd):
        if not self.mm_per_step:
            self.get_mm_per_step()
        step_cnt = int(run_dis / self.mm_per_step)
        step_us = int(((run_dis / run_spd) * 1000 * 1000) / step_cnt)
        acc_ctl_cnt = int(self.acc_ctl_mm / self.mm_per_step)
        return step_cnt, step_us, acc_ctl_cnt
    
    def get_best_rdy_z(self, rdy_x, rdy_y, base_pos=None):
        if not base_pos:
            base_pos = self.rdy_pos
        p_left = [base_pos[0][0], rdy_y, 0]
        p_right = [base_pos[2][0], rdy_y, 0]
        p_mid = [rdy_x, rdy_y, 0]
        p_left = self.get_linear2(base_pos[0], base_pos[1], p_left, False)
        p_right = self.get_linear2(base_pos[2], base_pos[3], p_right, False)
        p_mid = self.get_linear2(p_left, p_right, p_mid, True)
        self.print_msg('BEST_RDY_Z', "Src=%s, x=%.2f, y=%.2f, cal_z=%.2f" % (('RDY' if base_pos == self.rdy_pos else 'GAP'), rdy_x, rdy_y, p_mid[2]))
        return p_mid[2] if p_mid[2] < self.bed_max_err else self.bed_max_err
    
    def get_mm_per_step(self):
        if self.mm_per_step:
            return
        for stepper in self.toolhead.get_kinematics().get_steppers():
            if stepper.is_active_axis('z'):
                self.mm_per_step = self.step_base * stepper.get_step_dist()                
                self.print_msg('MM_PER_STEP', str(stepper.get_step_dist()))
        pass

    def run_to_next(self, nextpos, wait=False):
        nextpos_z = self.get_best_rdy_z(nextpos[0], nextpos[1]) + self.best_above_z
        self.move(nextpos[:2] + [nextpos_z], self.rdy_xy_spd, wait)
        pass

    def env_self_check(self, force=False):
        # 1. PR_ERR_CODE_SWAP_PIN_DETECTI
        self.write_swap_prtouch_cmd.send([self.pres_oid, 0])
        params0 = self.read_swap_prtouch_cmd.send([self.step_oid])
        self.write_swap_prtouch_cmd.send([self.pres_oid, 1])
        params1 = self.read_swap_prtouch_cmd.send([self.step_oid])       
        self.ck_and_raise_error(not params0 or not params1 or params0['sta'] != 0 or params1['sta'] != 1, 
                                PR_ERR_CODE_SWAP_PIN_DETECTI, [self.pres_swap_pin, self.step_swap_pin])
        self.print_msg('DEBUG', '--Self Test 1 = PR_ERR_CODE_SWAP_PIN_DETECTI, Pass!!--', force)

        if not self.need_self_check and not force:
            return   
            
        # 2. PR_ERR_CODE_PRES_READ_DATA_TIMEOUT
        self.deal_avgs_prtouch_cmd.send([self.pres_oid, 0])
        self.pres_res = []
        self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, 24])
        start_tick_s = time.time()
        while ((time.time() - start_tick_s) < (1.5 * (self.tri_acq_ms / 1000.) * 16)) and len(self.pres_res) < 16:
            self.delay_s(0.010)
        pnt_tick, pnt_vals = [], [[], [], [], []]
        for i in range(4, len(self.pres_res) - 4):
            pnt_tick.append(self.pres_res[i]['tick'] / 10000.)
            for j in range(self.pres_cnt):
                pnt_vals[j].append(self.pres_res[i]['ch%d' % j])
        tr = 0
        for i in range(1, len(pnt_tick)):
            tr += pnt_tick[i] - pnt_tick[i - 1]
        self.ck_and_raise_error(tr / (len(pnt_tick) - 1) > 2 * self.tri_acq_ms, PR_ERR_CODE_PRES_READ_DATA_TIMEOUT, [self.tri_acq_ms, tr / (len(pnt_tick) - 1)])
        self.print_msg('DEBUG', '--Self Test 2 = PR_ERR_CODE_PRES_READ_DATA_TIMEOUT, Pass!!--', force)
        
        # 3. PR_ERR_CODE_PRES_VAL_IS_CONSTANT
        for i in range(self.pres_cnt): 
            sums, avg = 0, sum(pnt_vals[i]) / len(pnt_vals[i])
            for j in range(len(pnt_vals[i])):
                pnt_vals[i][j] -= avg
                sums += abs(pnt_vals[i][j])              
            self.ck_and_raise_error(not sums, PR_ERR_CODE_PRES_VAL_IS_CONSTANT)
        self.print_msg('DEBUG', '--Self Test 3 = PR_ERR_CODE_PRES_VAL_IS_CONSTANT, Pass!!--', force)    

        # 4. PR_ERR_CODE_PRES_NOISE_TOO_BIG
        for i in range(self.pres_cnt):
            big_cnt = 0
            for j in range(len(pnt_vals[i])):
                big_cnt += (1 if abs(pnt_vals[i][j]) > (1000 if not self.use_adc else 200) else 0)
            self.ck_and_raise_error(big_cnt > len(pnt_vals[i]) / 2, PR_ERR_CODE_PRES_NOISE_TOO_BIG)
        self.print_msg('DEBUG', '--Self Test 4 = PR_ERR_CODE_PRES_NOISE_TOO_BIG, Pass!!--', force=force)    

        # 4. PR_ERR_CODE_PRES_NOT_BE_SENSED
        self.deal_avgs_prtouch_cmd.send([self.pres_oid, 16])
        now_pos = self.toolhead.get_position()
        self.toolhead.set_position(now_pos[:2] + [0, now_pos[3]], homing_axes=[2])
        self.pres_res = []
        self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, 32])
        self.shake_motor(self.shake_cnt)
        while ((time.time() - start_tick_s) < (1.5 * (self.tri_acq_ms / 1000.) * 16)) and len(self.pres_res) < 16:
            self.delay_s(0.010)
        pnt_tick, pnt_vals = [], [[], [], [], []]
        for i in range(4, len(self.pres_res) - 4):
            pnt_tick.append(self.pres_res[i]['tick'] / 10000.)
            for j in range(self.pres_cnt):
                pnt_vals[j].append(self.pres_res[i]['ch%d' % j])
        for i in range(self.pres_cnt):
            low_cnt = 0 
            for j in range(len(pnt_vals[i])):
                low_cnt += (1 if abs(pnt_vals[i][j]) < (200 if self.use_adc else 500) else 0)
            # self.ck_and_raise_error(low_cnt > len(pnt_vals[i]) / 2, PR_ERR_CODE_PRES_NOT_BE_SENSED)
        self.print_msg('DEBUG', '--Self Test 5 = PR_ERR_CODE_PRES_NOT_BE_SENSED, Pass!!--', force) 
        pass

    def probe_ready(self):
        if self.jump_probe_ready:
            self.jump_probe_ready = False
            return False
        self.ck_g28ed()
        for i in range(len(self.rdy_pos)):
            self.rdy_pos[i][2] = self.bed_max_err
        now_pos = self.toolhead.get_position()
        self.move(now_pos[:2] + [self.bed_max_err, now_pos[3]], self.rdy_z_spd)
        for i in range(4):
            self.move(self.rdy_pos[i], self.rdy_xy_spd)                
            self.rdy_pos[i][2] = self.run_step_prtouch(self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
            pass
        self.print_msg('RDY_POS', "[00=%.2f, 01=%.2f, 11=%.2f, 10=%.2f]" % (self.rdy_pos[0][2], self.rdy_pos[1][2], self.rdy_pos[2][2], self.rdy_pos[3][2]))
        return True

    def run_step_prtouch(self, down_min_z, probe_min_3err, rt_last=False, pro_cnt=3, crt_cnt=3, fast_probe=False, tri_min_hold=None, tri_max_hold=None):
        if not tri_min_hold or not tri_max_hold:
            tri_min_hold = self.tri_min_hold
            tri_max_hold = self.tri_max_hold
        res_z = []
        now_pos = self.toolhead.get_position()
        lost_min_cnt = 0   
        for i in range(pro_cnt):
            self.step_res, self.pres_res = [], []
            self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])
            step_cnt_down, step_us_down, acc_ctl_cnt = self.get_step_cnts(self.g29_down_min_z, self.tri_z_down_spd)
            step_us_down = int(step_us_down * 0.8) if (i == 0 and not self.use_adc) else step_us_down
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt, int(self.tri_hftr_cut*1000), int(self.tri_lftr_k1 * 1000), tri_min_hold, tri_max_hold])
            self.start_step_prtouch_cmd.send([self.step_oid, 0, self.tri_send_ms, step_cnt_down, step_us_down, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
            t_last = time.time()
            while (time.time() - t_last < (down_min_z / self.tri_z_down_spd + 5)) and (len(self.step_res) != MAX_BUF_LEN or len(self.pres_res) != MAX_BUF_LEN):
                self.delay_s(0.010)
            self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, 0, 0, 0, 0, 0, 0, 0])
            self.ck_and_raise_error(len(self.step_res) != MAX_BUF_LEN, PR_ERR_CODE_STEP_LOST_RUN_DATA, [len(self.step_res)])
            self.ck_and_raise_error(len(self.pres_res) != MAX_BUF_LEN, PR_ERR_CODE_PRES_LOST_RUN_DATA, [len(self.pres_res)])
            # self.print_res('DOWN=')
            step_par_down, pres_par_down, tri_time_down = [x for x in self.step_res], [x for x in self.pres_res], [self.pres_tri_time, self.step_tri_time]
            res_z.append(self.cal_tri_data(step_cnt_down, now_pos[2], step_par_down, pres_par_down, -lost_min_cnt * self.mm_per_step))
            
            can_rt = True if (i == 1 and not self.use_adc and math.fabs(res_z[0] - res_z[1]) < 0.05 and crt_cnt != pro_cnt) else False
            can_rt = True if can_rt else (i == (crt_cnt - 1)) and (max(res_z) - min(res_z) <= probe_min_3err)

            up_min_z = (step_cnt_down - self.step_res[-1]['step']) * self.mm_per_step
            step_cnt_up, step_us_up, acc_ctl_cnt = self.get_step_cnts(up_min_z, self.tri_z_up_spd)  
            step_cnt_up = int(step_cnt_down - self.step_res[-1]['step'])

            if fast_probe and pro_cnt > 2: 
                if i == 0 and up_min_z > self.best_above_z / 2:
                    _step_cnt_up = (self.best_above_z / 2) / self.mm_per_step
                    lost_min_cnt = step_cnt_up - _step_cnt_up
                    step_cnt_up = int(_step_cnt_up)
                elif i == pro_cnt - 1 or can_rt:
                    step_cnt_up += int(lost_min_cnt)

            self.step_res, self.pres_res = [], []
            self.start_step_prtouch_cmd.send([self.step_oid, 1, self.tri_send_ms, step_cnt_up, step_us_up, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
            # if can_rt: # and rt_last:
            #     break
            t_last = time.time()
            while (time.time() - t_last < (down_min_z / self.tri_z_down_spd + 5)) and (len(self.step_res) != MAX_BUF_LEN):
                self.delay_s(0.010)
            self.start_step_prtouch_cmd.send([self.step_oid, 1, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
            self.ck_and_raise_error(len(self.step_res) != MAX_BUF_LEN, PR_ERR_CODE_STEP_LOST_RUN_DATA, [len(self.step_res)])
            if can_rt:
                break
        res_z.sort()
        self.print_ary('RES_Z', res_z, len(res_z))
        return res_z[int((len(res_z) - 1) / 2)] if len(res_z) != 2 else (res_z[0] + res_z[1]) / 2

    def run_gap_prtouch(self, zero_pos=0, valid_ch=0):
        now_pos = self.toolhead.get_position()
        self.move(now_pos[:2] + [zero_pos + self.gap_dis_range / 2, now_pos[3]], self.rdy_z_spd)
        
        read_cnt = int(((self.gap_dis_range / self.z_gap_spd) + 5.) / (self.tri_acq_ms * 0.001))
        step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(self.gap_dis_range, self.z_gap_spd)
        acc_ctl_cnt = int(0) # action
        gap_zs = []
        for i in range(3):
            self.step_res, self.pres_res = [], []
            # 1. Down
            self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, read_cnt])            
            self.start_step_prtouch_cmd.send([self.step_oid, 0, self.tri_send_ms, step_cnt, step_us, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 1])
            t_last = time.time()
            while (time.time() - t_last < (self.gap_dis_range / self.tri_z_down_spd + 5)) and (len(self.step_res) != MAX_BUF_LEN):
                self.delay_s(0.010)
            self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
            self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, 0])      
            self.ck_and_raise_error(len(self.step_res) != MAX_BUF_LEN, PR_ERR_CODE_STEP_LOST_RUN_DATA, [len(self.step_res)])  
            self.ck_and_raise_error(len(self.pres_res) <= 0, PR_ERR_CODE_PRES_LOST_RUN_DATA, [len(self.pres_res)])
            # 3. Cal
            pres_tick, pres_data, pres_data_abs = [], [], []

            for i in range(len(self.pres_res)):
                pres_tick.append(self.pres_res[i]['tick'] / 10000.)
                pres_data.append(self.pres_res[i]['ch%d' % valid_ch])
                pres_data_abs.append(abs(pres_data[-1]))
            self.send_wave_tri(valid_ch, pres_data)

            mid_index = pres_data_abs.index(max(pres_data_abs))
            mid_index = mid_index if mid_index >= 1 else 1
            mid_index = mid_index if mid_index <= (len(pres_data_abs) - 2) else (len(pres_data_abs) - 2)
            mid_tick = pres_tick[mid_index]

            pres_tick_l, pres_tick_r = pres_tick[:mid_index], pres_tick[mid_index:][::-1]
            pres_data_l, pres_data_r = pres_data[:mid_index], pres_data[mid_index:][::-1]

            pres_data_l = [i * -1 for i in pres_data_l] if pres_data_l[0] > pres_data_l[-1] else pres_data_l
            pres_data_r = [i * -1 for i in pres_data_r] if pres_data_r[0] > pres_data_r[-1] else pres_data_r
            # self.print_ary('PRES_TICK_L', pres_tick_l, len(pres_tick_l), 3)
            # self.print_ary('PRES_DATA_L', pres_data_l, len(pres_data_l), 0)
            # self.print_ary('PRES_TICK_R', pres_tick_r, len(pres_tick_r), 3)            
            # self.print_ary('PRES_DATA_R', pres_data_r, len(pres_data_r), 0)

            min_val, max_val = min(pres_data_l), max(pres_data_l)
            for i in range(len(pres_data_l)):
                pres_data_l[i] = (pres_data_l[i] - min_val) / (max_val - min_val)
            # self.print_ary('NORM_PRES_DATA_L', pres_data_l, len(pres_data_l), 3)   
            angle = math.atan((pres_data_l[-1] - pres_data_l[0]) / len(pres_data_l))
            sin_angle, cos_angle = math.sin(-angle), math.cos(-angle)
            for i in range(len(pres_data_l)):
                pres_data_l[i] = (i - 0) * sin_angle + (pres_data_l[i] - 0) * cos_angle + 0  
            l_index = pres_data_l.index(min(pres_data_l))                
            # self.print_ary('CYCLE_PRES_DATA_L', pres_data_l, len(pres_data_l), 3)

            min_val, max_val = min(pres_data_r), max(pres_data_r)
            for i in range(len(pres_data_r)):
                pres_data_r[i] = (pres_data_r[i] - min_val) / (max_val - min_val)
            # self.print_ary('NORM_PRES_DATA_R',pres_data_r, len(pres_data_r), 3)    
            angle = math.atan((pres_data_r[-1] - pres_data_r[0]) / len(pres_data_r))
            sin_angle, cos_angle = math.sin(-angle), math.cos(-angle)
            for i in range(len(pres_data_r)):
                pres_data_r[i] = (i - 0) * sin_angle + (pres_data_r[i] - 0) * cos_angle + 0
            r_index = pres_data_r.index(min(pres_data_r))   
            # self.print_ary('CYCLE_PRES_DATA_R',  pres_data_r, len(pres_data_r), 3)    

            l_tick = pres_tick_l[l_index]
            r_tick = pres_tick_r[r_index]

            gap_z = ((r_tick - mid_tick) - (mid_tick - l_tick)) * self.z_gap_spd

            # self.print_msg('CAL_GAPS', 'l_tick=%f, mid_tick=%f, r_tick=%f, (r_tick - mid_tick)=%f, (mid_tick - l_tick)=%f, ((r_tick - mid_tick) - (mid_tick - l_tick))=%f, gap=%f' 
            #                % (l_tick, mid_tick, r_tick, (r_tick - mid_tick), (mid_tick - l_tick), ((r_tick - mid_tick) - (mid_tick - l_tick)), gap_z))

            gap_z = gap_z if gap_z <= 0.1 else 0.1
            gap_z = gap_z if gap_z >= 0.0 else 0.0

            gap_zs.append(gap_z)
        
        gap_zs.sort()
        self.print_ary('GAP_ZS', gap_zs, len(gap_zs), 3, False)

        self.move(now_pos, self.rdy_z_spd)
        return gap_zs[int(len(gap_zs) / 2)]

    def bed_mesh_post_proc(self, last_point):
        x_cnt = self.bed_mesh.bmc.mesh_config['x_count']
        y_cnt = self.bed_mesh.bmc.mesh_config['y_count']
        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max

        res1 = self.bed_mesh.bmc.probe_helper.results

        res1.append(last_point)
        for i in range(len(res1)):
            gap_z = 0 if not self.need_measure_gap else self.get_best_rdy_z(res1[i][0], res1[i][1], self.gap_pos)
            res1[i][2] += (gap_z + self.noz_ex_com)
            res1[i][2] += (1 - (res1[i][1] - min_y) / (max_y - min_y)) * (2 * self.tilt_corr_dis) - self.tilt_corr_dis

        mid_z = self.noz_ex_com
        if (x_cnt % 2 == 1) and (y_cnt % 2 == 1):
            mid_z = res1[int(len(res1) / 2)][2]
        else:
            if (x_cnt % 2 == 0) and (y_cnt % 2 == 0):
                axi = [[x_cnt / 2 - 1, y_cnt / 2 - 1], [x_cnt / 2, y_cnt / 2 - 1], [x_cnt / 2 - 1, y_cnt / 2, [x_cnt / 2, y_cnt / 2]]]
            elif x_cnt % 2 == 1:
                axi = [[int(x_cnt / 2), y_cnt / 2 - 1], [int(x_cnt / 2), y_cnt / 2]]
            elif y_cnt % 2 == 1:
                axi = [[x_cnt / 2 - 1, int(y_cnt / 2)], [x_cnt / 2, int(y_cnt / 2)]]

            mid_z = 0
            
            for i in range(len(axi)):
                mid_z += res1[int(axi[i][0] + axi[i][1] * x_cnt)][2]
            mid_z /= len(axi)

        if mid_z < self.noz_ex_com:
            mid_z = self.noz_ex_com - mid_z
            for i in range(len(res1)):
                res1[i][2] += mid_z

        del res1[-1]
        pass

    def check_bed_mesh(self, auto_g29=True):
        self.ck_g28ed()  
        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max
        mesh = self.bed_mesh.get_mesh()
        self.bed_mesh.set_mesh(None)
        self.probe_ready()
        self.jump_probe_ready = True
        if self.use_adc:
            self.set_fan_speed('heater_fan', self.fan_heat_min_spd)
        self.set_fan_speed('fan', 0.0)
        if mesh:
            self.bed_mesh.set_mesh(mesh)
            err_cnt, errs = 0, []
            for i in range(4):
                mesh_z = self.bed_mesh.z_mesh.calc_z(self.rdy_pos[i][0], self.rdy_pos[i][1]) - self.noz_ex_com - ((1 - (self.rdy_pos[i][1] - min_y) / (max_y - min_y)) * (2 * self.tilt_corr_dis) - self.tilt_corr_dis)
                errs.append(abs(self.rdy_pos[i][2] + self.gap_pos[i][2] - mesh_z))
                err_cnt += (1 if errs[i] > self.check_bed_mesh_max_err else 0)
                self.print_msg('CK_BED_MESH', 'P%d = [x=%.2f, y=%.2f, mest_z=%.2f, probe_z=%.2f, err_z=%.2f]' % (i, self.rdy_pos[i][0], self.rdy_pos[i][1], mesh_z, self.rdy_pos[i][2], errs[i]))    
            if err_cnt < 2:
                self.print_msg('DEBUG', "check_bed_mesh: Pass!!")
                return
            self.ck_and_raise_error(not auto_g29, PR_ERR_CODE_CK_BED_MESH_OUT_RANGE, [[errs[0], errs[1],errs[2], errs[3]], self.check_bed_mesh_max_err])
            self.print_msg('DEBUG', 'check_bed_mesh: Due to the great change of the hot bed, it needs to be re-leveled. ' + str([[errs[0], errs[1],errs[2], errs[3]], self.check_bed_mesh_max_err]))    
            pass
        if self.use_adc:
            self.set_fan_speed('heater_fan', self.fan_heat_max_spd)
        self.gcode.run_script_from_command('BED_MESH_CALIBRATE')
        self.gcode.run_script_from_command('CXSAVE_CONFIG')
        pass

    def clear_nozzle(self, hot_min_temp, hot_max_temp, bed_max_temp):
        tri_min_hold = self.tri_min_hold if self.use_adc else (self.tri_min_hold * 2)
        tri_max_hold = 3070 if self.use_adc else self.tri_max_hold
        min_x, min_y = self.clr_noz_start_x, self.clr_noz_start_y
        # max_x, max_y = self.clr_noz_start_x + self.clr_noz_len_x, self.clr_noz_start_y + self.clr_noz_len_y
        self.set_bed_temps(temp=bed_max_temp, wait=False)
        self.set_hot_temps(temp=hot_min_temp, wait=False, err=10)
        self.ck_g28ed()
        mesh = self.bed_mesh.get_mesh()
        self.bed_mesh.set_mesh(None)
        self.set_step_par(load_sys=False)
        random.seed(time.time())  
        cur_pos = self.toolhead.get_position()
        src_pos = [min_x + random.uniform(0, self.clr_noz_len_x - self.pa_clr_dis_mm - 5), 
                   min_y + random.uniform(0, self.clr_noz_len_y), self.bed_max_err, cur_pos[3]]
        end_pos = [src_pos[0] + self.pa_clr_dis_mm, src_pos[1], src_pos[2], src_pos[3]]
        self.set_hot_temps(temp=hot_min_temp, wait=True, err=10)
        self.move([end_pos[0], end_pos[1], end_pos[2]], self.rdy_xy_spd)
        if self.use_adc:
            self.set_fan_speed('heater_fan', self.fan_heat_min_spd)
        self.set_fan_speed('fan', 0.0)
        end_pos[2] = self.run_step_prtouch(self.g29_down_min_z, 0.1, False, 5, 3, True, tri_min_hold, tri_max_hold) 
        self.move([src_pos[0], src_pos[1], src_pos[2]], self.rdy_xy_spd)
        src_pos[2] = self.run_step_prtouch(self.g29_down_min_z, 0.1, False, 5, 3, True, tri_min_hold, tri_max_hold)
        if self.use_adc:
            self.set_fan_speed('heater_fan', self.fan_heat_max_spd)
        self.move([src_pos[0], src_pos[1], src_pos[2] - self.pa_clr_down_mm], self.tri_z_down_spd)
        self.set_hot_temps(temp=hot_max_temp, wait=True, err=10) 
        self.move(end_pos[:2] + [end_pos[2] + self.pa_clr_down_mm], self.clr_xy_spd)
        self.set_fan_speed('fan', 1.0)
        self.set_hot_temps(temp=hot_min_temp, wait=True, err=5)
        self.move([end_pos[0] + self.pa_clr_dis_mm, end_pos[1], end_pos[2] + self.bed_max_err], self.clr_xy_spd)
        self.set_fan_speed('fan', 0.)
        self.set_bed_temps(temp=bed_max_temp, wait=True, err=5)
        self.set_step_par(load_sys=True)
        self.bed_mesh.set_mesh(mesh)
        self.gcode.run_script_from_command('G28 Z')
        pass

    def thread_ovr(self):
        while self.ovr_det_hold:
            
            pass

    def run_G28_Z(self): 
        self.enable_steps()     
        self.get_mm_per_step()
        self.g29_cnt = 0
        self.jump_probe_ready = False
        self.toolhead.wait_moves()
        if not self.bed_mesh.get_mesh() and self.bed_mesh.pmgr.profiles.get('default', None):
            self.gcode.run_script_from_command('BED_MESH_PROFILE LOAD=\"default\"')
        mesh = self.bed_mesh.get_mesh()
        self.bed_mesh.set_mesh(None)   
        self.set_step_par(load_sys=False)
        now_pos = self.toolhead.get_position()    
        self.toolhead.set_position(now_pos[:2] + [0, now_pos[3]], homing_axes=[2])
        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max        
        now_pos = [min_x + (max_x - min_x) / 2, min_y + (max_y - min_y) / 2, now_pos[2], now_pos[3]]
        # 1. Check hot temp
        target_temp = self.heater_hot.target_temp
        if self.g28_wait_cool_down and self.heater_hot.smoothed_temp > (self.hot_min_temp + 5):
            self.print_msg('DEBUG', 'G28_Z: Wait for Nozzle to cool down[%.2f -> %.2f]...' % (target_temp, self.hot_min_temp))
            self.set_fan_speed('fan', 1.0)
            self.set_hot_temps(temp=self.hot_min_temp, wait=True, err=5) 
            self.set_fan_speed('fan', 0.0)
        # 2. First probe z
        random.seed(int(time.time()))
        now_pos0 = [now_pos[0] + (1 if (int(time.time() * 1000) % 2 == 0) else -1) * random.uniform(10, 20),
                    now_pos[1] + (1 if (int(time.time() * 100) % 2 == 0) else -1) * random.uniform(10, 20), 0, now_pos[3]]
        now_pos1 = [now_pos[0] + random.uniform(-1.0, +1.0), now_pos[1] + random.uniform(-1.0, +1.0), self.best_above_z * 2, now_pos[3]] 
        if self.use_adc:
            self.set_fan_speed('heater_fan', self.fan_heat_min_spd)
        self.set_fan_speed('fan', 0.0)
        self.move(now_pos0, self.rdy_xy_spd / 5)
        self.env_self_check()
        for i in range(10):
            self.step_res, self.pres_res = [], []
            params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])
            self.print_msg('AVGS_RESAULT', str(params))
            step_cnt_down, step_us_down, acc_ctl_cnt = self.get_step_cnts(self.max_z * 1.2, self.tri_z_down_spd * (1.2 if self.use_adc else 2.0))
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt, int(self.tri_hftr_cut*1000), int(self.tri_lftr_k1 * 1000), self.tri_min_hold, self.tri_max_hold])
            self.start_step_prtouch_cmd.send([self.step_oid, 0, self.tri_send_ms, step_cnt_down, step_us_down, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
            t_last = time.time()
            while (time.time() - t_last < (self.max_z * 2.0) / (self.tri_z_down_spd * 1.5)) and (len(self.step_res) != MAX_BUF_LEN or len(self.pres_res) != MAX_BUF_LEN):
                self.delay_s(0.010)
            self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
            self.start_pres_prtouch_cmd.send([self.pres_oid, 0, 0, 0, 0, 0, 0, 0, 0])
            self.ck_and_raise_error(len(self.step_res) != MAX_BUF_LEN, PR_ERR_CODE_STEP_LOST_RUN_DATA, [MAX_BUF_LEN, len(self.step_res)])
            self.ck_and_raise_error(len(self.pres_res) != MAX_BUF_LEN, PR_ERR_CODE_PRES_LOST_RUN_DATA, [MAX_BUF_LEN, len(self.pres_res)])
            out_mm = self.cal_tri_data(step_cnt_down, self.toolhead.get_position()[2], self.step_res, self.pres_res)            
            self.toolhead.set_position(now_pos0[:2] + [0, now_pos0[3]], homing_axes=[2])
            self.move(now_pos0[:2] + [self.best_above_z * 2, now_pos0[3]], self.tri_z_up_spd * (1.2 if self.use_adc else 2.0))
            if -0.5 < out_mm < 0.5:
                break
            self.ck_and_raise_error(i == 9, PR_ERR_CODE_G28_Z_DETECTION_TIMEOUT)
            
        # 3. Normal probe z
        self.move(now_pos1, self.rdy_xy_spd / 5)
        self.shake_motor(self.shake_cnt)
        res_z = self.run_step_prtouch(self.g29_down_min_z, self.probe_min_3err, False, 5, 5, True)
        self.toolhead.set_position(now_pos1[:2] + [self.toolhead.get_position()[2] - res_z, now_pos[3]], homing_axes=[2])
        self.move(now_pos[:2] + [self.bed_max_err, now_pos[3]], self.rdy_z_spd)
        # 4. Set hot temp to old target
        if self.g28_wait_cool_down:
            self.print_msg('DEBUG', 'G28_Z: Wait for Nozzle to recovery[%.2f -> %.2f]...' % (self.hot_min_temp, target_temp))
            self.set_hot_temps(temp=target_temp, wait=True if target_temp > self.hot_min_temp else False, err=5)
        if self.use_adc:
            self.set_fan_speed('heater_fan', self.fan_heat_max_spd)
        self.set_step_par(load_sys=True)
        self.bed_mesh.set_mesh(mesh)   
        pass

    def run_G29_Z(self):
        self.set_step_par(load_sys=False)
        x_cnt = self.bed_mesh.bmc.mesh_config['x_count'] 
        y_cnt = self.bed_mesh.bmc.mesh_config['y_count'] 
        self.toolhead.wait_moves()    
        now_pos = self.toolhead.get_position()
        if (int(self.g29_cnt) % int(x_cnt)) == 0:
            self.shake_motor(self.shake_cnt)

        if self.g29_cnt == 0:
            if self.use_adc:
                self.set_fan_speed('heater_fan', self.fan_heat_min_spd)
            self.set_fan_speed('fan', 0.0)
            if self.probe_ready():
                self.run_to_next(now_pos, True)
                self.shake_motor(self.shake_cnt)
        
        self.g29_cnt += 1
        
        now_pos[2] = self.run_step_prtouch(self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
        self.set_step_par(load_sys=True)

        if self.g29_cnt == x_cnt * y_cnt:
            self.g29_cnt = 0
            min_x, min_y = self.bed_mesh.bmc.mesh_min
            max_x, max_y = self.bed_mesh.bmc.mesh_max
            home_x = min_x + (max_x - min_x) / 2
            home_y = min_y + (max_y - min_y) / 2
            self.move([home_x, home_y, self.get_best_rdy_z(home_x, home_y, self.rdy_pos) + self.best_above_z], self.rdy_xy_spd)
            res_z = self.run_step_prtouch(self.g29_down_min_z, self.probe_min_3err, True, 5, 3, True)
            self.print_msg('CHECK_STEP_LOST', 'need=0, tri=%.2f' % res_z)
            self.ck_and_raise_error(abs(res_z) > self.lost_step_dis and self.lost_step_dis > 0, PR_ERR_CODE_HAVE_LOST_STEP)
            self.move([min_x + (max_x - min_x) / 2, min_y + (max_y - min_y) / 2, self.bed_max_err * 2], self.rdy_xy_spd)
            if self.use_adc:
                self.set_fan_speed('heater_fan', self.fan_heat_max_spd)
            self.bed_mesh_post_proc(now_pos)
        return now_pos

    cmd_READ_PRES_help = "Read The Press Vals."
    def cmd_READ_PRES(self, gcmd):
        self.pres_res = []
        read_cnt = gcmd.get_int('C', 1)
        self.read_pres_prtouch_cmd.send([self.pres_oid, self.tri_acq_ms, read_cnt])
        start_tick_s = time.time()
        while ((time.time() - start_tick_s) < (1.5 * (self.tri_acq_ms / 1000.) * read_cnt)) and len(self.pres_res) < read_cnt:
            self.delay_s(0.010)
        pnt_vals = [[], [], [], []]
        pnt_tick = []
        for i in range(len(self.pres_res)):
            pnt_tick.append(self.pres_res[i]['tick'] / 10000.)
            for j in range(self.pres_cnt):
                pnt_vals[j].append(self.pres_res[i]['ch%d' % j])

        self.print_ary('READ_PRES_TICKS', pnt_tick, len(pnt_tick), 3, True)        
        for i in range(self.pres_cnt):
            self.print_ary('READ_PRES_CH%d' % i, pnt_vals[i], len(pnt_vals[i]), 0, True)    
        pass

    cmd_TEST_SWAP_help = "Test The Swap Pin."
    def cmd_TEST_SWAP(self, gcmd):
        self.write_swap_prtouch_cmd.send([self.pres_oid, 0])
        params0 = self.read_swap_prtouch_cmd.send([self.step_oid])

        self.write_swap_prtouch_cmd.send([self.pres_oid, 1])
        params1 = self.read_swap_prtouch_cmd.send([self.step_oid])       
        if not params0 or not params1 or params0['sta'] != 0 or params1['sta'] != 1:
            self.print_msg('SWAP_TEST', '!!!Swap Test ERROR!!!', True)
        else:
            self.print_msg('SWAP_TEST', '---Swap Test Success---', True)
        pass 

    cmd_DEAL_AVGS_help = "Read And Cal The Avgs."
    def cmd_DEAL_AVGS(self, gcmd):
        read_cnt = gcmd.get_int('C', 8)
        params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, read_cnt])
        self.print_msg('AVGS_RESAULT', str(params), True)
        pass

    cmd_NOZZLE_CLEAR_help = "Clear the nozzle on bed."
    def cmd_NOZZLE_CLEAR(self, gcmd):
        hot_min_temp = gcmd.get_float('HOT_MIN_TEMP', self.hot_min_temp)
        hot_max_temp = gcmd.get_float('HOT_MAX_TEMP', self.hot_max_temp)
        bed_max_temp = gcmd.get_float('BED_MAX_TEMP', self.bed_max_temp)
        self.clear_nozzle(hot_min_temp, hot_max_temp, bed_max_temp)

    def change_hot_min_temp(self, temp):
        self.hot_min_temp = temp

    cmd_CHECK_BED_MESH_help = "Check the bed mesh."
    def cmd_CHECK_BED_MESH(self, gcmd):
        self.check_bed_mesh(gcmd.get_int('AUTO_G29', 0) > 0)
        pass

    cmd_START_STEP_PRTOUCH_help = "Start the step prtouch."
    def cmd_START_STEP_PRTOUCH(self, gcmd):
        self.enable_steps()
        self.get_mm_per_step()
        run_dir = gcmd.get_int('DIR', 0)
        run_spd = gcmd.get_float('SPD', 10)
        run_dis = gcmd.get_float('DIS', 10)
        self.step_res = []
        step_cnt, step_us, acc_ctl_cnt = self.get_step_cnts(run_dis, run_spd)
        self.start_step_prtouch_cmd.send([self.step_oid, run_dir, self.tri_send_ms, step_cnt, step_us, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
        t_last = time.time()
        while (time.time() - t_last < (run_dis / run_spd + 5)) and (len(self.step_res) != MAX_BUF_LEN):
            self.delay_s(0.010)
        self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
        self.ck_and_raise_error(len(self.step_res) != MAX_BUF_LEN, PR_ERR_CODE_STEP_LOST_RUN_DATA, [len(self.step_res)])
        pass

    cmd_PRTOUCH_READY_help = "Test the ready point."
    def cmd_PRTOUCH_READY(self, gcmd):
        self.probe_ready()
        pass

    cmd_SAFE_DOWN_Z_help = "Safe down z before G28"
    def cmd_SAFE_DOWN_Z(self, gcmd):
        self.get_mm_per_step()
        self.enable_steps()
        down_dis = gcmd.get_float('DOWN_DIS', 5)
        self.step_res, self.pres_res = [], []
        params = self.deal_avgs_prtouch_cmd.send([self.pres_oid, 8])
        self.print_msg('AVGS_RESAULT', str(params))
        step_cnt_down, step_us_down, acc_ctl_cnt = self.get_step_cnts(down_dis, self.tri_z_down_spd * (1.2 if self.use_adc else 2))
        self.start_pres_prtouch_cmd.send([self.pres_oid, 1, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt, int(self.tri_hftr_cut*1000), int(self.tri_lftr_k1 * 1000), self.tri_min_hold * 2, self.tri_max_hold])
        self.start_step_prtouch_cmd.send([self.step_oid, 1, self.tri_send_ms, step_cnt_down, step_us_down, acc_ctl_cnt, self.low_spd_nul, self.send_step_duty, 0])
        t_last = time.time()
        while (time.time() - t_last < (down_dis * 2.0) / (self.tri_z_down_spd * 1.5)) and (len(self.step_res) != MAX_BUF_LEN):
            self.delay_s(0.010)
        self.start_step_prtouch_cmd.send([self.step_oid, 1, 0, 0, 0, 0, self.low_spd_nul, self.send_step_duty, 0])
        self.start_pres_prtouch_cmd.send([self.pres_oid, 1, 0, 0, 0, 0, 0, 0, 0])
        self.ck_and_raise_error(len(self.step_res) != MAX_BUF_LEN, PR_ERR_CODE_STEP_LOST_RUN_DATA, [MAX_BUF_LEN, len(self.step_res)])
        pass

    cmd_TRIG_TEST_help = "Test The Tri is Normal"
    def cmd_TRIG_TEST(self, gcmd):
        self.gcode.run_script_from_command('SET_STEPPER_ENABLE STEPPER=stepper_x ENABLE=1')
        self.gcode.run_script_from_command('SET_STEPPER_ENABLE STEPPER=stepper_y ENABLE=1')        
        self.enable_steps()          
        self.get_mm_per_step()
        run_cnt = gcmd.get_int('C', 1)
        self.run_step_prtouch(self.g29_down_min_z, 0, False, run_cnt, run_cnt, True)
        pass

    cmd_SET_OVR_DET_help = "Overpressure detection"
    def cmd_SET_OVR_DET(self, gcmd):
        self.ovr_det_hold = gcmd.get_int('HOLD', 50000)
        if not self.ovr_det_hold:
            t = threading.Thread(target = self.thread_ovr, name = 'thread_ovr')
            t.start()
        pass

    cmd_SELF_CHECK_PRTOUCH_help = "Self check the pres."
    def cmd_SELF_CHECK_PRTOUCH(self, gcmd):
        self.env_self_check(force=True)
        pass

    cmd_TEST_PRTH_help = "For Debug Cmd"
    def cmd_TEST_PRTH(self, gcmd):
        # self.env_self_check(force=True)
        # self.print_msg('%f' % self.noz_ex_com)
        # zero_z = gcmd.get_float('Z', 0)
        # self.run_gap_prtouch(zero_z)
        # self.shake_motor(self.shake_cnt)
        # spd = gcmd.get_float('S', 0)
        # self.set_fan_speed('heater_fan', spd)
        self.set_fan_speed('fan', 1.0) 
        pass

def load_config(config):
    vrt = PRTouchEndstopWrapper(config)
    config.get_printer().add_object('probe', probe.PrinterProbe(config, vrt))
    return vrt

# /home/cc/klippy-env/bin/python3.10 /home/cc/klipper/klippy/klippy.py /home/cc/printer_data/config/printer.cfg -a /home/cc/printer_data/comms/klippy.sock -l /home/cc/printer_data/logs/klippy.log
# ./micropython /home/cc/klipper/klippy/klippy.py /home/cc/printer_data/config/printer.cfg -a /home/cc/printer_data/comms/klippy.sock -l /home/cc/printer_data/logs/klippy.log

# /home/cc/klippy-env/bin/python3.10 /home/cc/micropython_test/klipper/klippy/klippy.py /home/cc/printer_data/config/printer.cfg -a /home/cc/printer_data/comms/klippy.sock -l /home/cc/printer_data/logs/klippy.log
