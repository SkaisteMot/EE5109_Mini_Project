#!/usr/bin/env python3

import rospy
import numpy as np
import os
import csv
import time
import matplotlib.pyplot as plt
from std_msgs.msg import Float64, Bool, String
from sensor_msgs.msg import Joy, Imu
from geometry_msgs.msg import Vector3
import tf

# Import the custom data logger
from data_logger import DataLogger, ControllerAnalyzer, compare_controllers

# Set paths for imports
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, 'RobotController'))
sys.path.append(os.path.join(parent_dir, 'InverseKinematics'))

# Import our modified controllers
from PIDController_modified import PID_controller
from LQRController_modified import LQR_controller

class ControllerExperiment:
    """
    Class to run comparison experiments between PID and LQR controllers
    """
    def __init__(self):
        rospy.init_node('controller_experiment', anonymous=True)
        
        # Initialize controllers with logging enabled
        self.pid_controller = PID_controller(kp=0.75, ki=2.29, kd=0.0, logging_enabled=True)
        self.lqr_controller = LQR_controller(logging_enabled=True)
        
        # Current active controller
        self.active_controller = 'none'
        
        # Store current IMU data
        self.roll = 0.0
        self.pitch = 0.0
        
        # Experiment parameters
        self.experiment_duration = 20.0  # seconds per experiment
        self.disturbance_magnitude = 0.2  # radians
        
        # Subscribe to IMU topic to get roll and pitch
        rospy.Subscriber("notspot_imu/base_link_orientation", Imu, self.imu_callback)
        
        # Create publisher for controller type
        self.controller_pub = rospy.Publisher("controller_experiment/active_controller", 
                                              String, queue_size=10)
        
        # Create publisher to inject disturbances for testing
        self.disturbance_pub = rospy.Publisher("controller_experiment/disturbance", 
                                               Vector3, queue_size=10)
        
        # Directory for results
        self.results_dir = os.path.join(os.path.expanduser("~"), "controller_comparison_results")
        if not os.path.exists(self.results_dir):
            os.makedirs(self.results_dir)
            
        # Sleep to ensure publishers and subscribers are registered
        rospy.sleep(1.0)
        
    def imu_callback(self, msg):
        """
        Callback for IMU messages
        """
        q = msg.orientation
        rpy_angles = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.roll = rpy_angles[0]
        self.pitch = rpy_angles[1]
        
        # Run the active controller
        if self.active_controller == 'pid':
            self.pid_controller.run(self.roll, self.pitch)
        elif self.active_controller == 'lqr':
            self.lqr_controller.run(self.roll, self.pitch)
    
    def inject_disturbance(self, roll_dist=0.0, pitch_dist=0.0):
        """
        Inject a disturbance for testing controller response
        """
        disturbance = Vector3()
        disturbance.x = roll_dist
        disturbance.y = pitch_dist
        self.disturbance_pub.publish(disturbance)
        
    def run_pid_experiment(self, kp, ki, kd, duration=None):
        """
        Run an experiment with specific PID parameters
        """
        if duration is None:
            duration = self.experiment_duration
            
        # Configure PID controller
        self.pid_controller = PID_controller(kp=kp, ki=kd, kd=kd, logging_enabled=True)
        self.pid_controller.reset()
        self.active_controller = 'pid'
        
        # Publish active controller info
        self.controller_pub.publish("PID (kp={}, ki={}, kd={})".format(kp, ki, kd))
        
        # Run for specified duration
        rospy.loginfo("Starting PID experiment with kp={}, ki={}, kd={}".format(kp, ki, kd))
        start_time = rospy.Time.now()
        
        # Wait for initial settling (2 seconds)
        rospy.sleep(2.0)
        
        # Inject disturbance
        self.inject_disturbance(roll_dist=self.disturbance_magnitude, pitch_dist=0.0)
        rospy.loginfo("Injected roll disturbance of {} radians".format(self.disturbance_magnitude))
        
        # Continue experiment until duration is reached
        while (rospy.Time.now() - start_time).to_sec() < duration:
            if rospy.is_shutdown():
                break
            rospy.sleep(0.1)  # Sleep to avoid consuming CPU
        
        # Experiment finished
        self.active_controller = 'none'
        self.pid_controller.close_logger()
        rospy.loginfo("PID experiment completed")
        
        return self.pid_controller.logger.filename
    
    def run_lqr_experiment(self, q_values, r_values, gain_factor=0.18, duration=None):
        """
        Run an experiment with specific LQR parameters
        
        Args:
            q_values: List of 4 values for Q matrix diagonal [roll, roll_rate, pitch, pitch_rate]
            r_values: List of 2 values for R matrix diagonal [roll_control, pitch_control]
            gain_factor: Overall gain factor
            duration: Duration of experiment in seconds
        """
        if duration is None:
            duration = self.experiment_duration
            
        # Configure LQR controller
        self.lqr_controller = LQR_controller(logging_enabled=True)
        self.lqr_controller.set_parameters(q_values, r_values, gain_factor)
        self.lqr_controller.reset()
        self.active_controller = 'lqr'
        
        # Publish active controller info
        self.controller_pub.publish("LQR (Q={}, R={}, gain={})".format(q_values, r_values, gain_factor))
        
        # Run for specified duration
        rospy.loginfo("Starting LQR experiment with Q={}, R={}, gain={}".format(q_values, r_values, gain_factor))
        start_time = rospy.Time.now()
        
        # Wait for initial settling (2 seconds)
        rospy.sleep(2.0)
        
        # Inject disturbance
        self.inject_disturbance(roll_dist=self.disturbance_magnitude, pitch_dist=0.0)
        rospy.loginfo("Injected roll disturbance of {} radians".format(self.disturbance_magnitude))
        
        # Continue experiment until duration is reached
        while (rospy.Time.now() - start_time).to_sec() < duration:
            if rospy.is_shutdown():
                break
            rospy.sleep(0.1)  # Sleep to avoid consuming CPU
        
        # Experiment finished
        self.active_controller = 'none'
        self.lqr_controller.close_logger()
        rospy.loginfo("LQR experiment completed")
        
        return self.lqr_controller.logger.filename
    
    def run_pid_parameter_sweep(self):
        """
        Run experiments with different PID parameters
        """
        results = []
        
        # Different parameter combinations to test
        kp_values = [0.5, 0.75, 1.0]
        ki_values = [1.0, 2.29, 3.5]
        kd_values = [0.0, 0.1, 0.3]
        
        for kp in kp_values:
            for ki in ki_values:
                for kd in kd_values:
                    # Run the experiment
                    log_file = self.run_pid_experiment(kp, ki, kd)
                    
                    # Analyze results
                    analyzer = ControllerAnalyzer()
                    metrics = analyzer.calculate_metrics(log_file)
                    
                    # Store results
                    results.append({
                        'controller': 'pid',
                        'kp': kp,
                        'ki': ki,
                        'kd': kd,
                        'log_file': log_file,
                        'metrics': metrics
                    })
                    
                    # Short pause between experiments
                    rospy.sleep(2.0)
        
        # Save results to CSV
        self.save_pid_results_to_csv(results)
        
        return results
    
    def run_lqr_parameter_sweep(self):
        """
        Run experiments with different LQR parameters
        """
        results = []
        
        # Different parameter combinations to test
        q_roll_values = [10.0, 20.0, 30.0]
        q_pitch_values = [15.0, 25.0, 35.0]
        r_values = [0.5, 0.8, 1.2]
        gain_factors = [0.12, 0.18, 0.25]
        
        for q_roll in q_roll_values:
            for q_pitch in q_pitch_values:
                for r in r_values:
                    for gain in gain_factors:
                        # Create Q and R matrices
                        q_values = [q_roll, 1.0, q_pitch, 1.0]  # [roll, roll_rate, pitch, pitch_rate]
                        r_values = [r, r]  # [roll_control, pitch_control]
                        
                        # Run the experiment
                        log_file = self.run_lqr_experiment(q_values, r_values, gain)
                        
                        # Analyze results
                        analyzer = ControllerAnalyzer()
                        metrics = analyzer.calculate_metrics(log_file)
                        
                        # Store results
                        results.append({
                            'controller': 'lqr',
                            'q_roll': q_roll,
                            'q_pitch': q_pitch,
                            'r': r,
                            'gain': gain,
                            'log_file': log_file,
                            'metrics': metrics
                        })
                        
                        # Short pause between experiments
                        rospy.sleep(2.0)
        
        # Save results to CSV
        self.save_lqr_results_to_csv(results)
        
        return results
    
    def run_controller_comparison(self):
        """
        Run a direct comparison between best PID and best LQR configurations
        """
        # Best PID configuration (you would determine this from parameter sweep)
        best_pid_kp = 0.75
        best_pid_ki = 2.29
        best_pid_kd = 0.1
        
        # Best LQR configuration (you would determine this from parameter sweep)
        best_lqr_q = [20.0, 1.0, 25.0, 1.0]
        best_lqr_r = [0.8, 0.8]
        best_lqr_gain = 0.18
        
        # Run PID experiment
        pid_log_file = self.run_pid_experiment(best_pid_kp, best_pid_ki, best_pid_kd)
        
        # Run LQR experiment
        lqr_log_file = self.run_lqr_experiment(best_lqr_q, best_lqr_r, best_lqr_gain)
        
        # Compare the results
        comparison_file = os.path.join(self.results_dir, "pid_vs_lqr_comparison.csv")
        comparison = compare_controllers(pid_log_file, lqr_log_file, comparison_file)
        
        # Create visualizations
        self.create_comparison_plots(pid_log_file, lqr_log_file)
        
        return comparison
    
    def save_pid_results_to_csv(self, results):
        """
        Save PID parameter sweep results to CSV
        """
        filename = os.path.join(self.results_dir, "pid_parameter_sweep_results.csv")
        
        # Determine all metrics keys
        all_metrics = set()
        for result in results:
            all_metrics.update(result['metrics'].keys())
        
        # Create headers
        headers = ['controller', 'kp', 'ki', 'kd', 'log_file'] + list(all_metrics)
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            
            for result in results:
                row = [
                    result['controller'],
                    result['kp'],
                    result['ki'],
                    result['kd'],
                    result['log_file']
                ]
                
                # Add metrics
                for metric in all_metrics:
                    row.append(result['metrics'].get(metric, ""))
                
                writer.writerow(row)
        
        rospy.loginfo(f"PID parameter sweep results saved to {filename}")
    
    def save_lqr_results_to_csv(self, results):
        """
        Save LQR parameter sweep results to CSV
        """
        filename = os.path.join(self.results_dir, "lqr_parameter_sweep_results.csv")
        
        # Determine all metrics keys
        all_metrics = set()
        for result in results:
            all_metrics.update(result['metrics'].keys())
        
        # Create headers
        headers = ['controller', 'q_roll', 'q_pitch', 'r', 'gain', 'log_file'] + list(all_metrics)
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            
            for result in results:
                row = [
                    result['controller'],
                    result['q_roll'],
                    result['q_pitch'],
                    result['r'],
                    result['gain'],
                    result['log_file']
                ]
                
                # Add metrics
                for metric in all_metrics:
                    row.append(result['metrics'].get(metric, ""))
                
                writer.writerow(row)
        
        rospy.loginfo(f"LQR parameter sweep results saved to {filename}")
    
    def create_comparison_plots(self, pid_file, lqr_file):
        """
        Create comparison plots between PID and LQR controllers
        """
        # Load data from CSV files
        pid_data = self.load_csv_data(pid_file)
        lqr_data = self.load_csv_data(lqr_file)
        
        # Error over time plot
        self.plot_error_comparison(pid_data, lqr_data)
        
        # Control effort plot
        self.plot_control_effort_comparison(pid_data, lqr_data)
        
        # Error vs. control effort plot (efficiency)
        self.plot_efficiency_comparison(pid_data, lqr_data)
    
    def load_csv_data(self, csv_file):
        """
        Load data from a CSV file
        """
        data = {}
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for col in reader.fieldnames:
                data[col] = []
            
            for row in reader:
                for col in reader.fieldnames:
                    try:
                        data[col].append(float(row[col]) if row[col] else 0.0)
                    except ValueError:
                        data[col].append(row[col])
        
        return data
    
    def plot_error_comparison(self, pid_data, lqr_data):
        """
        Plot roll and pitch errors for PID vs LQR
        """
        plt.figure(figsize=(12, 8))
        
        # Roll error plot
        plt.subplot(2, 1, 1)
        plt.plot(pid_data['time'], pid_data['roll_error'], label='PID Roll Error')
        plt.plot(lqr_data['time'], lqr_data['roll_error'], label='LQR Roll Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Roll Error (rad)')
        plt.title('Roll Error Comparison')
        plt.grid(True)
        plt.legend()
        
        # Pitch error plot
        plt.subplot(2, 1, 2)
        plt.plot(pid_data['time'], pid_data['pitch_error'], label='PID Pitch Error')
        plt.plot(lqr_data['time'], lqr_data['pitch_error'], label='LQR Pitch Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Pitch Error (rad)')
        plt.title('Pitch Error Comparison')
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.results_dir, 'pid_vs_lqr_error_comparison.png'))
        plt.close()
    
    def plot_control_effort_comparison(self, pid_data, lqr_data):
        """
        Plot control effort for PID vs LQR
        """
        plt.figure(figsize=(10, 6))
        
        # Get the right column names for control output
        pid_output_col = 'total_output_mag' if 'total_output_mag' in pid_data else 'roll_output'
        lqr_output_col = 'control_magnitude' if 'control_magnitude' in lqr_data else 'control_roll'
        
        plt.plot(pid_data['time'], pid_data[pid_output_col], label='PID Control Effort')
        plt.plot(lqr_data['time'], lqr_data[lqr_output_col], label='LQR Control Effort')
        plt.xlabel('Time (s)')
        plt.ylabel('Control Effort')
        plt.title('Control Effort Comparison')
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.results_dir, 'pid_vs_lqr_control_effort_comparison.png'))
        plt.close()
    
    def plot_efficiency_comparison(self, pid_data, lqr_data):
        """
        Plot error vs control effort (efficiency)
        """
        plt.figure(figsize=(10, 6))
        
        # Get column names
        pid_output_col = 'total_output_mag' if 'total_output_mag' in pid_data else 'roll_output'
        lqr_output_col = 'control_magnitude' if 'control_magnitude' in lqr_data else 'control_roll'
        
        # Calculate total error for each time point
        pid_total_error = [abs(r) + abs(p) for r, p in zip(pid_data['roll_error'], pid_data['pitch_error'])]
        lqr_total_error = [abs(r) + abs(p) for r, p in zip(lqr_data['roll_error'], lqr_data['pitch_error'])]
        
        plt.scatter(pid_total_error, pid_data[pid_output_col], label='PID', alpha=0.5)
        plt.scatter(lqr_total_error, lqr_data[lqr_output_col], label='LQR', alpha=0.5)
        plt.xlabel('Total Error (|roll| + |pitch|)')
        plt.ylabel('Control Effort')
        plt.title('Control Efficiency Comparison (Lower is Better)')
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.results_dir, 'pid_vs_lqr_efficiency_comparison.png'))
        plt.close()

    def plot_pid_parameter_effects(self, results):
        """
        Create plots showing how different PID parameters affect performance
        """
        # Extract data
        kp_values = []
        ki_values = []
        kd_values = []
        settling_times = []
        max_errors = []
        control_efforts = []
        
        for result in results:
            kp_values.append(result['kp'])
            ki_values.append(result['ki'])
            kd_values.append(result['kd'])
            
            metrics = result['metrics']
            settling_times.append(metrics.get('roll_settling_time', 0) or 0)
            max_errors.append(metrics.get('roll_max_error', 0) or 0)
            control_efforts.append(metrics.get('total_control_effort', 0) or 0)
        
        # Create 3D plot for kp, ki, kd vs settling time
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        scatter = ax.scatter(kp_values, ki_values, kd_values, 
                            c=settling_times, cmap='viridis', 
                            s=50, alpha=0.8)
        
        ax.set_xlabel('kp')
        ax.set_ylabel('ki')
        ax.set_zlabel('kd')
        plt.colorbar(scatter, label='Settling Time (s)', shrink=0.5)
        plt.title('Effect of PID Parameters on Settling Time')
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.results_dir, 'pid_parameter_effects_settling_time.png'))
        plt.close()
        
        # Create plots for kp vs performance metrics
        plt.figure(figsize=(15, 5))
        
        plt.subplot(1, 3, 1)
        plt.scatter(kp_values, settling_times, c=ki_values, cmap='plasma')
        plt.xlabel('kp')
        plt.ylabel('Settling Time (s)')
        plt.colorbar(label='ki')
        plt.grid(True)
        
        plt.subplot(1, 3, 2)
        plt.scatter(kp_values, max_errors, c=ki_values, cmap='plasma')
        plt.xlabel('kp')
        plt.ylabel('Maximum Error')
        plt.colorbar(label='ki')
        plt.grid(True)
        
        plt.subplot(1, 3, 3)
        plt.scatter(kp_values, control_efforts, c=ki_values, cmap='plasma')
        plt.xlabel('kp')
        plt.ylabel('Control Effort')
        plt.colorbar(label='ki')
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.results_dir, 'pid_kp_performance_effects.png'))
        plt.close()

def main():
    experiment = ControllerExperiment()
    
    # Run a full parameter sweep (uncomment to use)
    # pid_results = experiment.run_pid_parameter_sweep()
    # lqr_results = experiment.run_lqr_parameter_sweep()
    # experiment.plot_pid_parameter_effects(pid_results)
    
    # Run direct comparison between best PID and LQR configurations
    experiment.run_controller_comparison()
    
    rospy.loginfo("Controller comparison completed")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass