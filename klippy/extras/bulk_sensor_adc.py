# Load Cell Endstop Support
#
# Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Generic interface for Bulk Sensor ADCs that can supply high frequency
# data from the MCU.
class BulkSensorAdc(object):
    # return the MCU the sensor is attached to
    def get_mcu(self):
        return None
    # get the number of samples per second that the sensor is configured for
    def get_samples_per_second(self):
        return 1
    # returns a tuple of the minimum and maximum value of the sensor, used to
    # detect if a data value is saturated
    def get_range(self):
        return -1, 1
    # subscribe to data being published from the sensor
    # callback is called with a list containing (time, counts) tuples:
    # [(time, counts), (time, counts), ...]
    # The callback returns True to continue receiving data and False to stop
    def add_client(self, callback):
        pass

# Interface for BulkSensorAdc variants that want to support LoadCellProbe
class LoadCellEndstopSensor(object):
    # create an endstop oid on the MCU where the sensor is and attach
    # it to the sensor
    def attach_endstop(self, endstop_oid):
        pass

# Helper for ClockSyncRegression that handles generating timestamps
# while processing a batch of samples
#TODO: REVIEW: I want to move this to bulk_sensor
class TimestampHelper:
    def __init__(self, clock_sync, clock_updater, samples_per_msg):
        self.clock_sync = clock_sync
        self.last_sequence = clock_updater.get_last_sequence()
        self.time_base, self.chip_base, self.inv_freq = (
            clock_sync.get_time_translation())
        self.samples_per_msg = samples_per_msg
        self.msg_cdiff = None
        self.seq = 0
        self.last_msg_index = 0
    def update_sequence(self, sequence):
        seq_diff = (sequence - self.last_sequence) & 0xffff
        seq_diff -= (seq_diff & 0x8000) << 1
        self.seq = self.last_sequence + seq_diff
        self.msg_cdiff = self.seq * self.samples_per_msg - self.chip_base
    def time_of_msg(self, msg_index):
        self.last_msg_index = msg_index
        time = self.time_base + (self.msg_cdiff + msg_index) * self.inv_freq
        return round(time, 6)
    def set_last_chip_clock(self):
        chip_clock = self.seq * self.samples_per_msg + self.last_msg_index
        self.clock_sync.set_last_chip_clock(chip_clock)
