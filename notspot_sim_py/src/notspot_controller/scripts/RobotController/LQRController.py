import numpy as np
import rospy

class LQR_controller(object):
    def __init__(self):
        # desired roll and pitch angles
        self.desired_roll_pitch = np.array([0.0, 0.0])
        self.dt = 0.02  # Default time step
        
        # Physical parameters from URDF
        # From the base_link inertia in URDF: ixx="0.00058474", iyy="0.00029699"
        self.inertia_roll = 0.00058474   # Roll moment of inertia from URDF
        self.inertia_pitch = 0.00029699  # Pitch moment of inertia from URDF
        
        # Damping coefficients - REDUCED for more responsive movement
        self.damping_roll = 0.05     # Natural damping coefficient for roll (was 0.15)
        self.damping_pitch = 0.05    # Natural damping coefficient for pitch (was 0.15)
        
        # Control output smoothing - REDUCED for faster response
        self.last_u = np.array([0.0, 0.0])
        self.control_alpha = 0.15    # Smoothing factor for control outputs (was 0.3)
        
        # Cost matrices - INCREASED to give stronger corrections
        self.Q = np.diag([30.0, 1.5, 35.0, 1.5])  # [roll, roll_rate, pitch, pitch_rate] (was [20, 1, 25, 1])
        self.R = np.diag([0.6, 0.6])              # Control effort penalty (was [0.8, 0.8])
        
        # Internal state
        self.last_error = np.array([0.0, 0.0])
        self.estimated_vel = np.array([0.0, 0.0])
        self.last_time = rospy.Time.now()
        self.derivative_alpha = 0.15  # Smoothing for derivatives (was 0.2)
        
        # Initialize matrices with default dt
        self.update_matrices(self.dt)
        
        # Precompute gain for default dt
        self.K = self.compute_lqr_gain()
        
        self.max_output = 1.0    # Control output limits
        self.gain_factor = 0.3   # INCREASED gain factor for stronger control (was 0.18)
        
        # Debug - Print initialization
        rospy.logwarn("LQR Controller initialized with optimized parameters")
        rospy.logwarn(f"Inertias - Roll: {self.inertia_roll}, Pitch: {self.inertia_pitch}")
        rospy.logwarn(f"Damping - Roll: {self.damping_roll}, Pitch: {self.damping_pitch}")
        rospy.logwarn(f"Gain factor: {self.gain_factor} (increased from 0.18)")
        rospy.logwarn(f"Control smoothing: {self.control_alpha} (reduced from 0.3)")
    
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
        
        # Apply smoothing to control outputs - less smoothing for faster response
        u = self.control_alpha * u_raw + (1 - self.control_alpha) * self.last_u
        self.last_u = u

        # Debug print - more detailed
        if abs(roll) > 0.01 or abs(pitch) > 0.01:
            rospy.loginfo(f"[LQR] Roll={roll:.4f}, Pitch={pitch:.4f}, Error=[{current_error[0]:.4f}, {current_error[1]:.4f}]")
            rospy.loginfo(f"[LQR] Raw Control={u_raw}, Final Control={u}")

        # Clip output
        u = np.clip(u, -self.max_output, self.max_output)

        if abs(u[0]) > 0.1 or abs(u[1]) > 0.1:
            rospy.logwarn(f"[LQR] Large Control: {u} - Active stabilization")

        return u
    
    def reset(self):
        rospy.logwarn("[LQR] Controller reset")
        self.last_time = rospy.Time.now()
        self.last_error = np.array([0.0, 0.0])
        self.estimated_vel = np.array([0.0, 0.0])
        self.last_u = np.array([0.0, 0.0])
    
    def desired_RP_angles(self, des_roll, des_pitch):
        self.desired_roll_pitch = np.array([des_roll, des_pitch])
        rospy.loginfo(f"[LQR] New target orientation: Roll={des_roll:.4f}, Pitch={des_pitch:.4f}")