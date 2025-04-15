import numpy as np
import rospy

class LQR_controller(object):
    def __init__(self):
        # desired roll and pitch angles (no yaw)
        self.desired_roll_pitch = np.array([0.0, 0.0])
        
        # LQR parameters
        # State cost matrix Q (penalizes state error)
        self.Q = np.array([
            [0.639, 0, 0, 0],  # Penalize roll error
            [0, 1.0, 0, 0],    # Penalize pitch error
            [0, 0, 0.1, 0],    # Penalize roll velocity error
            [0, 0, 0, 0.1]     # Penalize pitch velocity error
        ])
        
        # Control input cost matrix R (penalizes control effort)
        self.R = np.array([
            [0.01, 0],        # Penalty for roll correction effort
            [0, 0.01]         # Penalty for pitch correction effort
        ])
        
        # State transition matrix A (how the state evolves on its own)
        # For a simple model: [roll, pitch, roll_vel, pitch_vel]
        self.A = np.array([
            [1.0, 0, 0.02, 0],    # roll += roll_vel * dt
            [0, 1.0, 0, 0.02],    # pitch += pitch_vel * dt 
            [0, 0, 1.0, 0],       # roll_vel stays constant without input
            [0, 0, 0, 1.0]        # pitch_vel stays constant without input
        ])
        
        # Input matrix B (how control inputs affect state)
        self.B = np.array([
            [0, 0],           # control doesn't directly affect position
            [0, 0],           # control doesn't directly affect position
            [0.02, 0],        # roll acceleration caused by control
            [0, 0.02]         # pitch acceleration caused by control
        ])
        
        # For storing previous state
        self.last_error = np.array([0.0, 0.0])
        self.last_error_vel = np.array([0.0, 0.0])
        self.last_time = rospy.Time.now()
        
        self.K = None
        self.dt = 0.02  # Assuming 50Hz control loop
    
    def compute_lqr_gain(self):
        # Number of iterations for approximation
        N = 50
        
        # Create a list of N + 1 elements for P matrices
        P = [None] * (N + 1)
        
        # Final state cost
        P[N] = self.Q
        
        # Backward recursion to solve for P
        for i in range(N, 0, -1):
            # Discrete-time algebraic Riccati equation
            AT_P = self.A.T @ P[i]
            AT_P_B = AT_P @ self.B
            BT_P_B_R = self.B.T @ P[i] @ self.B + self.R
            BT_P_B_R_inv = np.linalg.pinv(BT_P_B_R)
            
            P[i-1] = self.Q + (self.A.T @ P[i] @ self.A) - (AT_P_B @ BT_P_B_R_inv @ AT_P_B.T)
        
        # Compute optimal feedback gain K
        BT_P1_B_R = self.B.T @ P[1] @ self.B + self.R
        BT_P1_B_R_inv = np.linalg.pinv(BT_P1_B_R)
        K = -BT_P1_B_R_inv @ self.B.T @ P[1] @ self.A
        
        return K
    
    def run(self, roll, pitch):
        # Calculate time step
        t_now = rospy.Time.now()
        dt = (t_now - self.last_time).to_sec()
        self.last_time = t_now
        
        # If dt is too large or too small, use default
        if dt < 0.001 or dt > 0.1:
            dt = 0.02
        
        # Update A and B matrices with actual dt
        self.A[0, 2] = dt
        self.A[1, 3] = dt
        self.B[2, 0] = dt
        self.B[3, 1] = dt
        
        # Estimate velocity (derivative of error)
        current_error = self.desired_roll_pitch - np.array([roll, pitch])
        error_vel = (current_error - self.last_error) / dt
        
        # Construct current state vector [roll_error, pitch_error, roll_vel, pitch_vel]
        state = np.concatenate((current_error, error_vel))
        
        # Compute LQR gain matrix K if not already computed
        if self.K is None or np.any(np.isnan(self.K)):
            self.K = self.compute_lqr_gain()
        
        # Compute optimal control input: u = K * x
        u = self.K @ state
        
        # Store current error for next iteration
        self.last_error = current_error
        
        return u
    
    def reset(self):
        self.last_time = rospy.Time.now()
        self.last_error = np.array([0.0, 0.0])
        self.last_error_vel = np.array([0.0, 0.0])
        self.K = None
    
    def desired_RP_angles(self, des_roll, des_pitch):
        self.desired_roll_pitch = np.array([des_roll, des_pitch])