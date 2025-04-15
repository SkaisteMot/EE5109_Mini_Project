import numpy as np
import rospy

class LQR_controller(object):
    def __init__(self):
        # desired roll and pitch angles
        self.desired_roll_pitch = np.array([0.0, 0.0])
        self.dt = 0.02  # Assuming 50Hz control loop

        self.A_single = np.array([
            [1.0, self.dt],
            [0.0, 1.0]
        ])
        self.B_single = np.array([
            [0.0],
            [self.dt]
        ])

        # Build full state-space for roll + pitch
        self.A = np.block([
            [self.A_single, np.zeros((2, 2))],
            [np.zeros((2, 2)), self.A_single]
        ])
        self.B = np.block([
            [self.B_single, np.zeros((2, 1))],
            [np.zeros((2, 1)), self.B_single]
        ])
        
        # Cost matrices
        self.Q = np.diag([2.0, 0.5, 2.0, 0.5])  # [roll, roll_vel, pitch, pitch_vel]
        self.R = np.diag([0.05, 0.05])           # penalize control effort
        
        # Internal state
        self.last_error = np.array([0.0, 0.0])
        self.estimated_vel = np.array([0.0, 0.0])
        self.last_time = rospy.Time.now()
        self.derivative_alpha = 0.2
        
        self.K = self.compute_lqr_gain()

        self.max_output = 1.0
    
    def compute_lqr_gain(self):   
        """Solves discrete-time Riccati equation for LQR gain."""
        # Create a list of N + 1 elements for P matrices
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
        self.last_time = t_now

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
        u = -self.K @ state

        # Debug print
        rospy.loginfo(f"[LQR] State: {state}, Control: {u}")

        # Clip output
        u = np.clip(u, -self.max_output, self.max_output)

        # Debug print
        rospy.loginfo(f"[LQR] State: {state}, Control: {u}")

        return u
    
    def reset(self):
        self.last_time = rospy.Time.now()
        self.last_error = np.array([0.0, 0.0])
        self.last_error_vel = np.array([0.0, 0.0])
    
    def desired_RP_angles(self, des_roll, des_pitch):
        self.desired_roll_pitch = np.array([des_roll, des_pitch])