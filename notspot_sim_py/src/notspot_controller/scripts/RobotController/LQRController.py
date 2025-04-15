import numpy as np
import rospy

class LQR_controller(object):
    def __init__(self):
        # desired roll and pitch angles
        self.desired_roll_pitch = np.array([0.0, 0.0])
        self.dt = 0.02  # Default time step
        
        # Physical parameters (you may need to tune these)
        self.inertia_roll = 0.01   # Estimated moment of inertia for roll (kg*m²)
        self.inertia_pitch = 0.01  # Estimated moment of inertia for pitch (kg*m²)
        self.damping_roll = 0.1    # Natural damping coefficient for roll
        self.damping_pitch = 0.1   # Natural damping coefficient for pitch
        
        # Control output smoothing
        self.last_u = np.array([0.0, 0.0])
        self.control_alpha = 0.4  # Smoothing factor for control outputs
        
        # Cost matrices - penalize deviations from desired state
        self.Q = np.diag([10.0, 1.0, 10.0, 1.0])  # [roll, roll_rate, pitch, pitch_rate]
        self.R = np.diag([0.5, 0.5])              # penalize control effort
        
        # Internal state
        self.last_error = np.array([0.0, 0.0])
        self.estimated_vel = np.array([0.0, 0.0])
        self.last_time = rospy.Time.now()
        self.derivative_alpha = 0.15  # Smoothing for derivatives
        
        # Initialize matrices with default dt
        self.update_matrices(self.dt)
        
        # Precompute gain for default dt
        self.K = self.compute_lqr_gain()
        
        self.max_output = 1.0  # Control output limits
        self.gain_factor = 0.5  # Reduced from 0.8 to be less aggressive
    
    def update_matrices(self, dt):
        """Update A and B matrices with current dt and physical model"""
        # A matrix for roll dynamics [angle, angular_rate]
        roll_A = np.array([
            [1.0, dt],  # angle += angular_rate * dt
            [0, 1.0 - (self.damping_roll * dt / self.inertia_roll)]  # angular_rate decays with damping
        ])
        
        # B matrix for roll control input
        roll_B = np.array([
            [0],  # control doesn't directly affect angle
            [dt / self.inertia_roll]  # control torque affects angular acceleration
        ])
        
        # A matrix for pitch dynamics [angle, angular_rate]
        pitch_A = np.array([
            [1.0, dt],  # angle += angular_rate * dt
            [0, 1.0 - (self.damping_pitch * dt / self.inertia_pitch)]  # angular_rate decays with damping
        ])
        
        # B matrix for pitch control input
        pitch_B = np.array([
            [0],  # control doesn't directly affect angle
            [dt / self.inertia_pitch]  # control torque affects angular acceleration
        ])

        # Build full state-space model
        self.A = np.block([
            [roll_A, np.zeros((2, 2))],
            [np.zeros((2, 2)), pitch_A]
        ])
        self.B = np.block([
            [roll_B, np.zeros((2, 1))],
            [np.zeros((2, 1)), pitch_B]
        ])
    
    def compute_lqr_gain(self):   
        """Solves discrete-time Riccati equation for LQR gain."""
        P = self.Q.copy()
        for _ in range(100):  # Iterative DARE solver
            BT_P = self.B.T @ P
            K_temp = np.linalg.inv(self.R + BT_P @ self.B) @ BT_P @ self.A
            P = self.Q + self.A.T @ P @ (self.A - self.B @ K_temp)
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
        
        rospy.loginfo(f"dt = {dt}")
        
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

        # Debug print
        rospy.loginfo(f"[LQR] Raw Control: {state}, Control: {u}")

        # Clip output
        u = np.clip(u, -self.max_output, self.max_output)

        # Debug print
        rospy.loginfo(f"[LQR] Clipped Control: {u}")

        return u
    
    def reset(self):
        self.last_time = rospy.Time.now()
        self.last_error = np.array([0.0, 0.0])
        self.estimated_vel = np.array([0.0, 0.0])
        self.last_u = np.array([0.0, 0.0])
    
    def desired_RP_angles(self, des_roll, des_pitch):
        self.desired_roll_pitch = np.array([des_roll, des_pitch])