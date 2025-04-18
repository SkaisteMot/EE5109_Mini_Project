import numpy as np
import rospy
import os
import sys

# Add the parent directory to the Python path to import the data_logger module
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from .data_logger import DataLogger

class LQR_controller(object):
    def __init__(self, logging_enabled=True):
        # desired roll and pitch angles
        self.desired_roll_pitch = np.array([0.0, 0.0])
        self.dt = 0.02  # Default time step
        
        # Physical parameters from URDF
        # From the base_link inertia in URDF: ixx="0.00058474", iyy="0.00029699"
        self.inertia_roll = 0.00058474   # Roll moment of inertia from URDF
        self.inertia_pitch = 0.00029699  # Pitch moment of inertia from URDF
        
        # Damping coefficients based on physical properties
        self.damping_roll = 0.15    # Natural damping coefficient for roll
        self.damping_pitch = 0.15   # Natural damping coefficient for pitch
        
        # Control output smoothing
        self.last_u = np.array([0.0, 0.0])
        self.control_alpha = 0.3    # Smoothing factor for control outputs 
        # Cost matrices - tuned for the actual robot inertia
        self.Q = np.diag([20.0, 1.0, 25.0, 1.0])  # [roll, roll_rate, pitch, pitch_rate]
        self.R = np.diag([0.8, 0.8])              # Control effort penalty
        
        # Internal state
        self.last_error = np.array([0.0, 0.0])
        self.estimated_vel = np.array([0.0, 0.0])
        self.last_time = rospy.Time.now()
        self.derivative_alpha = 0.2  # Smoothing for derivatives
        
        # Initialize matrices with default dt
        self.update_matrices(self.dt)
        
        # Precompute gain for default dt
        self.K = self.compute_lqr_gain()
        
        self.max_output = 1.0    # Control output limits
        self.gain_factor = 0.18  # Adjusted based on actual inertia values 
        
        # For data collection
        self.logging_enabled = logging_enabled
        if self.logging_enabled:
            q_values = f"Q_{self.Q[0,0]}_{self.Q[2,2]}"
            r_values = f"R_{self.R[0,0]}"
            self.logger = DataLogger(controller_type=f"lqr_{q_values}_{r_values}")
            self.logger.initialize_file([
                'time', 'roll', 'pitch', 'roll_error', 'pitch_error',
                'roll_vel', 'pitch_vel', 'state_vector',
                'control_roll', 'control_pitch', 'control_magnitude',
                'raw_control_roll', 'raw_control_pitch',
                'inertia_roll', 'inertia_pitch', 'dt'
            ])
            self.start_time = rospy.Time.now()
    
    def update_matrices(self, dt):
        # A matrix for state dynamics [roll, roll_rate, pitch, pitch_rate]
        self.A = np.array([
            [1.0, dt,  0.0, 0.0],  # roll += roll_rate * dt
            [0.0, 1.0 - (self.damping_roll * dt / self.inertia_roll), 0.0, 0.0],  # roll_rate
            [0.0, 0.0, 1.0, dt],  # pitch += pitch_rate * dt
            [0.0, 0.0, 0.0, 1.0 - (self.damping_pitch * dt / self.inertia_pitch)]  # pitch_rate
        ])
        
        # B matrix maps control inputs to state changes
        # Scaled for the robot's actual inertia values
        self.B = np.array([
            [0.0, 0.0],  # control doesn't directly affect roll angle
            [dt / self.inertia_roll, 0.0],  # first input affects roll acceleration
            [0.0, 0.0],  # control doesn't directly affect pitch angle
            [0.0, dt / self.inertia_pitch]  # second input affects pitch acceleration
        ])
    
    def compute_lqr_gain(self):   
        """Solves discrete-time Riccati equation for LQR gain."""
        P = self.Q.copy()
        # Iterative DARE solver with convergence check
        for _ in range(100):  
            BT_P = self.B.T @ P
            K_temp = np.linalg.inv(self.R + BT_P @ self.B) @ BT_P @ self.A
            P_new = self.Q + self.A.T @ P @ (self.A - self.B @ K_temp)
            
            # Check for convergence
            if np.allclose(P, P_new, rtol=1e-5):
                break
                
            P = P_new
            
        K = np.linalg.inv(self.R + self.B.T @ P @ self.B) @ self.B.T @ P @ self.A
        return K
    
    def run(self, roll, pitch):
        # Calculate time step
        t_now = rospy.Time.now()
        dt = (t_now - self.last_time).to_sec()
        self.last_time = t_now
        
        # If dt is too large or too small, use default
        if dt < 0.001 or dt > 0.1:
            dt = self.dt
        
        # Update matrices with current dt
        self.update_matrices(dt)
        
        # Only recompute K if dt has changed significantly (optimization)
        if abs(dt - self.dt) > 0.005:
            self.K = self.compute_lqr_gain()
            self.dt = dt

        # Compute angle error
        current_error = self.desired_roll_pitch - np.array([roll, pitch])

        # Estimate velocity using smoothed numerical differentiation
        raw_vel = (current_error - self.last_error) / dt
        self.estimated_vel = (
            self.derivative_alpha * raw_vel +
            (1 - self.derivative_alpha) * self.estimated_vel
        )

        # Update last error
        self.last_error = current_error

        # Construct full state vector: [roll_err, roll_vel, pitch_err, pitch_vel]
        state = np.array([
            current_error[0], self.estimated_vel[0],
            current_error[1], self.estimated_vel[1]
        ])

        # Compute control effort
        u_raw = -self.gain_factor * (self.K @ state)
        
        # Apply smoothing to control outputs
        u = self.control_alpha * u_raw + (1 - self.control_alpha) * self.last_u
        self.last_u = u

        # Clip output
        u_clipped = np.clip(u, -self.max_output, self.max_output)
        
        # Log data if enabled
        if self.logging_enabled:
            elapsed_time = (t_now - self.start_time).to_sec()
            control_magnitude = np.linalg.norm(u_clipped)
            state_str = ','.join([str(val) for val in state])
            
            log_data = {
                'time': elapsed_time,
                'roll': roll,
                'pitch': pitch,
                'roll_error': current_error[0],
                'pitch_error': current_error[1],
                'roll_vel': self.estimated_vel[0],
                'pitch_vel': self.estimated_vel[1],
                'state_vector': state_str,
                'control_roll': u_clipped[0],
                'control_pitch': u_clipped[1],
                'control_magnitude': control_magnitude,
                'raw_control_roll': u_raw[0],
                'raw_control_pitch': u_raw[1],
                'inertia_roll': self.inertia_roll,
                'inertia_pitch': self.inertia_pitch,
                'dt': dt
            }
            self.logger.log_data(log_data)

        return u_clipped
    
    def reset(self):
        self.last_time = rospy.Time.now()
        self.last_error = np.array([0.0, 0.0])
        self.estimated_vel = np.array([0.0, 0.0])
        self.last_u = np.array([0.0, 0.0])
        
        # Reset logging start time
        if self.logging_enabled:
            self.start_time = rospy.Time.now()
    
    def desired_RP_angles(self, des_roll, des_pitch):
        self.desired_roll_pitch = np.array([des_roll, des_pitch])
        
    def set_parameters(self, q_diag, r_diag, gain_factor=None):
        """
        Adjust the controller parameters for experiments
        
        Args:
            q_diag: 4-element list/array of diagonal values for Q matrix
            r_diag: 2-element list/array of diagonal values for R matrix
            gain_factor: Optional scaling factor for the control output
        """
        self.Q = np.diag(q_diag)
        self.R = np.diag(r_diag)
        
        if gain_factor is not None:
            self.gain_factor = gain_factor
            
        # Recompute the gain matrix
        self.K = self.compute_lqr_gain()
        
        # Update logger filename if it exists
        if self.logging_enabled:
            q_values = f"Q_{self.Q[0,0]}_{self.Q[2,2]}"
            r_values = f"R_{self.R[0,0]}"
            self.logger.close()
            self.logger = DataLogger(controller_type=f"lqr_{q_values}_{r_values}")
            self.logger.initialize_file([
                'time', 'roll', 'pitch', 'roll_error', 'pitch_error',
                'roll_vel', 'pitch_vel', 'state_vector',
                'control_roll', 'control_pitch', 'control_magnitude',
                'raw_control_roll', 'raw_control_pitch',
                'inertia_roll', 'inertia_pitch', 'dt'
            ])
            self.start_time = rospy.Time.now()
    
    def close_logger(self):
        if self.logging_enabled and hasattr(self, 'logger'):
            self.logger.close()