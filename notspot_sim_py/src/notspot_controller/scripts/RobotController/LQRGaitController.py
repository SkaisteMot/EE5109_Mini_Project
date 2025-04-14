import numpy as np
import rospy
from RoboticsUtilities.Transformations import rotxyz, rotz
from . PIDController import PID_controller
from . GaitController import GaitController

class LQRGaitController(GaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
        self.use_imu = use_imu
        self.use_button = True
        
        # Same contact phases as trot gait
        contact_phases = np.array([[1, 1, 1, 0],  # 0: Leg swing
                                  [1, 0, 1, 1],  # 1: Moving stance forward
                                  [1, 0, 1, 1],  
                                  [1, 1, 1, 0]])

        z_error_constant = 0.02 * 4
        z_leg_lift = 0.07

        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)

        # Similar velocities as trot but might want to tune these differently
        self.max_x_velocity = 0.020 #[m/s] 
        self.max_y_velocity = 0.015 #[m/s]
        self.max_yaw_rate = 0.5 #[rad/s]

        # LQR gains (Q and R matrices diagonals)
        # State weights (position, velocity, orientation)
        self.Q = np.diag([10.0, 10.0, 10.0, 8.0, 8.0, 8.0])
        # Control input weights
        self.R = np.diag([1.0, 1.0, 1.0, 1.0])
        
        # System dynamics matrices (to be filled in)
        self.A = np.eye(6)  # State transition matrix
        self.B = np.zeros((6, 4))  # Control input matrix
        
        # Pre-compute the LQR gain matrix K
        self.K = self.compute_lqr_gain()
        
        # Use same swing and stance controllers as trot for now
        self.swingController = LQRSwingController(self.stance_ticks, self.swing_ticks, 
                                                self.time_step, self.phase_length, 
                                                z_leg_lift, self.default_stance, self.K)
        
        self.stanceController = LQRStanceController(self.phase_length, self.stance_ticks, 
                                                  self.swing_ticks, self.time_step, 
                                                  z_error_constant, self.K)
        
        # For stability control
        self.pid_controller = PID_controller(0.15, 0.02, 0.002)
        
    def compute_lqr_gain(self):
        """Compute the LQR gain matrix K"""
        # Simplified LQR calculation - in a real implementation, 
        # you would solve the Riccati equation
        # This is just a placeholder with reasonable values
        return np.array([
            [0.5, 0.0, 0.0, 0.7, 0.0, 0.0],
            [0.0, 0.5, 0.0, 0.0, 0.7, 0.0],
            [0.0, 0.0, 0.5, 0.0, 0.0, 0.7],
            [0.3, 0.0, 0.0, 0.4, 0.0, 0.0]
        ])
    
    def updateStateCommand(self, msg, state, command):
        # Similar to trot gait but might want to process differently
        command.velocity[0] = msg.axes[4] * self.max_x_velocity  # Forward/backward
        command.velocity[1] = msg.axes[0] * self.max_y_velocity  # Left/right
        command.yaw_rate = msg.axes[2] * self.max_yaw_rate  # Rotation
        
        # Height control
        state.body_local_position[2] = msg.axes[1] * 0.03
        
        # Button handling 
        if self.use_button:
            if msg.buttons[7]:
                self.use_imu = not self.use_imu
                self.use_button = False
                rospy.loginfo(f"LQR Controller - Use roll/pitch compensation: {self.use_imu}")
            
        if not self.use_button:
            if not msg.buttons[7]:
                self.use_button = True

    def step(self, state, command):
        # Get the current contact modes for each leg
        contact_modes = self.contacts(state.ticks)
        
        # Initialize new foot locations
        new_foot_locations = np.zeros((3, 4))
        
        # Current state vector (position, velocity, orientation)
        x = np.array([
            state.body_local_position[0],
            state.body_local_position[1],
            state.body_local_orientation[2],  # yaw
            command.velocity[0],
            command.velocity[1],
            command.yaw_rate
        ])
        
        # Compute LQR control input u = -Kx
        u = -np.dot(self.K, x)
        
        # Process each leg
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            if contact_mode == 1:  # Stance phase
                new_location = self.stanceController.next_foot_location(leg_index, state, command, u)
            else:  # Swing phase
                swing_proportion = float(self.subphase_ticks(state.ticks)) / float(self.swing_ticks)
                new_location = self.swingController.next_foot_location(swing_proportion, leg_index, state, command, u)
                
            new_foot_locations[:, leg_index] = new_location
            
        # Apply IMU-based stability control if enabled
        if self.use_imu:
            compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
            roll_compensation = -compensation[0]
            pitch_compensation = -compensation[1]
            
            rot = rotxyz(roll_compensation, pitch_compensation, 0)
            new_foot_locations = np.matmul(rot, new_foot_locations)
            
        state.ticks += 1
        return new_foot_locations
        
    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        state.robot_height = command.robot_height
        return state.foot_locations

class LQRSwingController(object):
    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance, K):
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.phase_length = phase_length
        self.z_leg_lift = z_leg_lift
        self.default_stance = default_stance
        self.K = K  # LQR gain matrix
        
    def raibert_touchdown_location(self, leg_index, command, u):
        # Enhanced touchdown computation using LQR control input
        delta_pos_2d = command.velocity * self.phase_length * self.time_step
        
        # Apply LQR correction to touchdown location
        lqr_correction = np.array([u[0], u[1], 0]) * 0.01  # Scale factor for control input
        
        delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0]) + lqr_correction
        
        theta = self.stance_ticks * self.time_step * command.yaw_rate
        rotation = rotz(theta)
        
        return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos
        
    def swing_height(self, swing_phase):
        if swing_phase < 0.5:
            swing_height_ = swing_phase / 0.5 * self.z_leg_lift
        else:
            swing_height_ = self.z_leg_lift * (1 - (swing_phase - 0.5) / 0.5)
        return swing_height_
        
    def next_foot_location(self, swing_prop, leg_index, state, command, u):
        assert swing_prop >= 0 and swing_prop <= 1
        foot_location = state.foot_locations[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command, u)
        
        time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)
        
        velocity = (touchdown_location - foot_location) / float(time_left) * np.array([1, 1, 0])
        
        delta_foot_location = velocity * self.time_step
        z_vector = np.array([0, 0, swing_height_ + command.robot_height])
        
        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location

class LQRStanceController(object):
    def __init__(self, phase_length, stance_ticks, swing_ticks, time_step, z_error_constant, K):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant
        self.K = K  # LQR gain matrix
        
    def position_delta(self, leg_index, state, command, u):
        z = state.foot_locations[2, leg_index]
        
        # Basic step distances
        step_dist_x = command.velocity[0] * (float(self.phase_length) / self.swing_ticks)
        step_dist_y = command.velocity[1] * (float(self.phase_length) / self.swing_ticks)
        
        # Apply LQR control corrections
        step_dist_x += u[0] * 0.01  # LQR x correction
        step_dist_y += u[1] * 0.01  # LQR y correction
        
        velocity = np.array([
            -(step_dist_x / 4) / (float(self.time_step) * self.stance_ticks),
            -(step_dist_y / 4) / (float(self.time_step) * self.stance_ticks),
            1.0 / self.z_error_constant * (state.robot_height - z)
        ])
        
        delta_pos = velocity * self.time_step
        
        # Apply yaw rate correction from LQR
        yaw_rate_correction = command.yaw_rate + u[2] * 0.01
        delta_ori = rotz(-yaw_rate_correction * self.time_step)
        
        return (delta_pos, delta_ori)
        
    def next_foot_location(self, leg_index, state, command, u):
        foot_location = state.foot_locations[:, leg_index]
        (delta_pos, delta_ori) = self.position_delta(leg_index, state, command, u)
        next_foot_location = np.matmul(delta_ori, foot_location) + delta_pos
        return next_foot_location