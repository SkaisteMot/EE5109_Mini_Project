#!/usr/bin/env python3


import csv
import os
import rospy
import numpy as np
from datetime import datetime

class DataLogger:
    """
    Utility class for logging controller performance data to CSV files
    """
    def __init__(self, log_dir="controller_logs", controller_type="unknown"):
        """
        Initialize the data logger
        
        Args:
            log_dir: Directory to store log files
            controller_type: Type of controller ('pid', 'lqr', or other identifier)
        """
        self.controller_type = controller_type
        
        # Create log directory if it doesn't exist
        self.log_dir = os.path.join(os.path.expanduser("~"), log_dir)
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        # Generate a unique filename based on timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(self.log_dir, f"{controller_type}_{timestamp}.csv")
        
        # Initialize the CSV file with headers
        self.initialized = False
        self.file = None
        self.writer = None
        
        # Data buffers
        self.buffer_size = 100  # Save to file every 100 records
        self.data_buffer = []
        
        rospy.loginfo(f"Data logger initialized. Logging to {self.filename}")
    
    def initialize_file(self, headers):
        """
        Initialize the CSV file with column headers
        
        Args:
            headers: List of column headers
        """
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(headers)
        self.initialized = True
        self.headers = headers
    
    def log_data(self, data_row):
        """
        Log a row of data to the CSV file
        
        Args:
            data_row: List or dictionary of values to log
        """
        # If data_row is a dictionary, convert to list based on headers
        if isinstance(data_row, dict):
            if not self.initialized:
                # Auto-initialize with dictionary keys as headers
                self.initialize_file(list(data_row.keys()))
            row = [data_row.get(h, "") for h in self.headers]
        else:
            # If data_row is a list, use as is
            if not self.initialized:
                # Auto-initialize with generic column names
                generic_headers = [f"col_{i}" for i in range(len(data_row))]
                self.initialize_file(generic_headers)
            row = data_row
        
        # Add to buffer
        self.data_buffer.append(row)
        
        # Flush buffer if it's full
        if len(self.data_buffer) >= self.buffer_size:
            self.flush()
    
    def flush(self):
        """
        Write buffered data to file
        """
        if self.initialized and self.data_buffer:
            self.writer.writerows(self.data_buffer)
            self.file.flush()
            self.data_buffer = []
    
    def close(self):
        """
        Close the log file
        """
        if self.initialized:
            self.flush()  # Write any remaining data
            self.file.close()
            rospy.loginfo(f"Data logger closed. Data saved to {self.filename}")

class ControllerAnalyzer:
    """
    Utility class for analyzing controller performance
    """
    def __init__(self):
        """
        Initialize the analyzer
        """
        pass
    
    @staticmethod
    def calculate_metrics(data_file):
        """
        Calculate performance metrics from logged data
        
        Args:
            data_file: Path to CSV data file
            
        Returns:
            Dictionary of calculated metrics
        """
        # Read the CSV file
        data = []
        headers = []
        with open(data_file, 'r') as f:
            reader = csv.reader(f)
            headers = next(reader)  # Get headers
            for row in reader:
                data.append([float(val) if val else 0.0 for val in row])
        
        # Convert to numpy array for analysis
        data = np.array(data)
        
        # Find column indices for relevant data
        time_idx = headers.index('time') if 'time' in headers else 0
        roll_error_idx = headers.index('roll_error') if 'roll_error' in headers else None
        pitch_error_idx = headers.index('pitch_error') if 'pitch_error' in headers else None
        control_output_idx = headers.index('control_output') if 'control_output' in headers else None
        
        metrics = {}
        
        # Calculate metrics if we have error data
        if roll_error_idx is not None and pitch_error_idx is not None:
            roll_errors = data[:, roll_error_idx]
            pitch_errors = data[:, pitch_error_idx]
            
            # Mean Absolute Error (MAE)
            metrics['roll_mae'] = np.mean(np.abs(roll_errors))
            metrics['pitch_mae'] = np.mean(np.abs(pitch_errors))
            
            # Root Mean Square Error (RMSE)
            metrics['roll_rmse'] = np.sqrt(np.mean(np.square(roll_errors)))
            metrics['pitch_rmse'] = np.sqrt(np.mean(np.square(pitch_errors)))
            
            # Maximum Absolute Error
            metrics['roll_max_error'] = np.max(np.abs(roll_errors))
            metrics['pitch_max_error'] = np.max(np.abs(pitch_errors))
            
            # Settling time (time to reach within 2% of final value)
            # This is an approximation, assumes errors should converge to zero
            threshold = 0.02
            for i in range(len(roll_errors)):
                if i > 10 and all(abs(err) < threshold for err in roll_errors[i:i+10]):
                    metrics['roll_settling_time'] = data[i, time_idx] - data[0, time_idx]
                    break
            else:
                metrics['roll_settling_time'] = None
                
            for i in range(len(pitch_errors)):
                if i > 10 and all(abs(err) < threshold for err in pitch_errors[i:i+10]):
                    metrics['pitch_settling_time'] = data[i, time_idx] - data[0, time_idx]
                    break
            else:
                metrics['pitch_settling_time'] = None
        
        # Calculate control effort if available
        if control_output_idx is not None:
            control_outputs = data[:, control_output_idx]
            metrics['total_control_effort'] = np.sum(np.abs(control_outputs))
            metrics['mean_control_effort'] = np.mean(np.abs(control_outputs))
            metrics['max_control_effort'] = np.max(np.abs(control_outputs))
        
        return metrics

def compare_controllers(pid_file, lqr_file, output_file):
    """
    Compare PID and LQR controller performance and save results
    
    Args:
        pid_file: Path to PID controller data
        lqr_file: Path to LQR controller data
        output_file: Path to save comparison results
    """
    analyzer = ControllerAnalyzer()
    pid_metrics = analyzer.calculate_metrics(pid_file)
    lqr_metrics = analyzer.calculate_metrics(lqr_file)
    
    # Combine and save comparison
    comparison = {
        'metric': [],
        'pid_value': [],
        'lqr_value': [],
        'difference': [],
        'percent_improvement': []
    }
    
    for metric in set(pid_metrics.keys()).union(lqr_metrics.keys()):
        pid_val = pid_metrics.get(metric, None)
        lqr_val = lqr_metrics.get(metric, None)
        
        if pid_val is not None and lqr_val is not None:
            diff = lqr_val - pid_val
            if pid_val != 0:
                pct_improvement = (pid_val - lqr_val) / pid_val * 100
            else:
                pct_improvement = 0
                
            comparison['metric'].append(metric)
            comparison['pid_value'].append(pid_val)
            comparison['lqr_value'].append(lqr_val)
            comparison['difference'].append(diff)
            comparison['percent_improvement'].append(pct_improvement)
    
    # Save comparison to CSV
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(comparison.keys())
        for i in range(len(comparison['metric'])):
            writer.writerow([comparison[key][i] for key in comparison.keys()])
    
    return comparison