#!/usr/bin/env python3
# Author: lnotspotl

import rospy
import numpy as np
import os
import sys

# Add the parent directory to the Python path to import the data_logger module
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from data_logger import DataLogger

class PID_controller(object):
    def __init__(self, kp, ki, kd, logging_enabled=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # desired roll and pitch angles
        # (we don't really care about yaw)
        self.desired_roll_pitch = np.array([0.0, 0.0])

        self.I_term = np.array([0.0, 0.0])
        self.D_term = np.array([0.0, 0.0])

        # TODO: Tune max_I
        self.max_I = 0.2
        self.last_error = np.array([0.0, 0.0])
        
        # For data collection
        self.logging_enabled = logging_enabled
        if self.logging_enabled:
            self.logger = DataLogger(controller_type=f"pid_kp{kp}_ki{ki}_kd{kd}")
            self.logger.initialize_file([
                'time', 'roll', 'pitch', 'roll_error', 'pitch_error',
                'roll_P', 'roll_I', 'roll_D', 'pitch_P', 'pitch_I', 'pitch_D',
                'roll_output', 'pitch_output', 'total_output_mag'
            ])
            self.start_time = rospy.Time.now()

    def run(self, roll, pitch):
        # determine error
        error = self.desired_roll_pitch - np.array([roll, pitch])

        # determine time step
        t_now = rospy.Time.now()
        step = (t_now - self.last_time).to_sec()

        # I term update
        self.I_term = self.I_term + error * step

        # anti-windup
        for i in range(2):
            if(self.I_term[i] < -self.max_I):
                self.I_term[i] = -self.max_I
            elif(self.I_term[i] > self.max_I):
                self.I_term[i] = self.max_I

        # approximate first derivate
        self.D_term = (error - self.last_error) / step

        # update last values 
        self.last_time = t_now
        self.last_error = error

        # compute return values
        P_ret = self.kp * error
        I_ret = self.I_term * self.ki
        D_ret = self.D_term * self.kd
        
        # Calculate final output
        output = P_ret + I_ret + D_ret

        # Log data if enabled
        if self.logging_enabled:
            elapsed_time = (t_now - self.start_time).to_sec()
            total_output_mag = np.linalg.norm(output)
            
            log_data = {
                'time': elapsed_time,
                'roll': roll,
                'pitch': pitch,
                'roll_error': error[0],
                'pitch_error': error[1],
                'roll_P': P_ret[0],
                'roll_I': I_ret[0],
                'roll_D': D_ret[0],
                'pitch_P': P_ret[1],
                'pitch_I': I_ret[1],
                'pitch_D': D_ret[1],
                'roll_output': output[0],
                'pitch_output': output[1],
                'total_output_mag': total_output_mag
            }
            self.logger.log_data(log_data)

        return output

    def reset(self):
        self.last_time = rospy.Time.now()
        self.I_term = np.array([0.0, 0.0])
        self.D_term = np.array([0.0, 0.0])
        self.last_error = np.array([0.0, 0.0])
        
        # Reset logging start time
        if self.logging_enabled:
            self.start_time = rospy.Time.now()

    def desired_RP_angles(self, des_roll, des_pitch):
        # set desired roll and pitch angles
        self.desired_roll_pitch = np.array([des_roll, des_pitch])
        
    def close_logger(self):
        if self.logging_enabled and hasattr(self, 'logger'):
            self.logger.close()