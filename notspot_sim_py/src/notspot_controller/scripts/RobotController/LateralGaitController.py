#!/usr/bin/env python3
import numpy as np
from .GaitController import GaitController
from RoboticsUtilities.Transformations import rotxyz, rotz

class LateralGaitController(GaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step):
        contact_phases = np.array([[1, 1, 0, 1],  # Left swing
                                   [0, 1, 1, 1],  # Right swing
                                   [1, 1, 0, 1],  
                                   [0, 1, 1, 1]])

        z_leg_lift = 0.07  # Adjust based on testing

        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)

        self.max_y_velocity = 0.02  # Sideways movement speed [m/s]

        self.swingController = LateralSwingController(self.stance_ticks, self.swing_ticks, self.time_step,
                                                      self.phase_length, z_leg_lift, self.default_stance)

        self.stanceController = LateralStanceController(self.phase_length, self.stance_ticks, self.swing_ticks,
                                                        self.time_step)

    def step(self, state, command):
        contact_modes = self.contacts(state.ticks)

        new_foot_locations = np.zeros((3, 4))
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            if contact_mode == 1:
                new_location = self.stanceController.next_foot_location(leg_index, state, command)
            else:
                swing_proportion = float(self.subphase_ticks(state.ticks)) / float(self.swing_ticks)
                new_location = self.swingController.next_foot_location(swing_proportion, leg_index, state, command)

            new_foot_locations[:, leg_index] = new_location

        state.ticks += 1
        return new_foot_locations

class LateralSwingController(object):
    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.phase_length = phase_length
        self.z_leg_lift = z_leg_lift
        self.default_stance = default_stance

    def swing_height(self, swing_phase):
        if swing_phase < 0.5:
            return swing_phase / 0.5 * self.z_leg_lift
        return self.z_leg_lift * (1 - (swing_phase - 0.5) / 0.5)

    def next_foot_location(self, swing_prop, leg_index, state, command):
        foot_location = state.foot_locations[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        delta_pos = np.array([0, command.velocity[1] * self.time_step, 0])  # Sideways movement
        return foot_location + delta_pos + np.array([0, 0, swing_height_])

class LateralStanceController(object):
    def __init__(self, phase_length, stance_ticks, swing_ticks, time_step):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step

    def next_foot_location(self, leg_index, state, command):
        foot_location = state.foot_locations[:, leg_index]
        delta_pos = np.array([0, -command.velocity[1] * self.time_step, 0])  # Sideways movement
        return foot_location + delta_pos
