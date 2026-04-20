#!/usr/bin/env python3
"""IMU data preprocessor for before Madgwick filter.

Uses calibration data to remove biases, filter noise, and improve accuracy.
"""

import numpy as np
from typing import Dict, Optional, Tuple
from collections import deque
from pathlib import Path


class IMUPreprocessor:
    """Preprocesses raw IMU sensor data using calibration data."""
    
    def __init__(self, calibration_data_dir: Optional[str] = None, logger=None):
        """Initialize preprocessor with optional calibration data.
        
        Args:
            calibration_data_dir: Directory containing angular_imu.csv and linear_imu.csv
            logger: ROS2 logger for debug messages
        """
        self.logger = logger
        self.calibration_data_dir = calibration_data_dir or '/shared'
        
        # Bias estimates (mean values from calibration)
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        
        # Standard deviation for covariance calculation
        self.accel_std = np.array([1.0, 1.0, 1.0])
        self.gyro_std = np.array([1.0, 1.0, 1.0])
        
        # Low-pass filter buffers (moving average)
        self.filter_window_size = 5
        self.accel_buffer = deque(maxlen=self.filter_window_size)
        self.gyro_buffer = deque(maxlen=self.filter_window_size)
        
        # Load calibration data if available
        self.load_calibration_data()
        
    def load_calibration_data(self):
        """Load calibration data from CSV files and compute bias/std."""
        try:
            linear_path = Path(self.calibration_data_dir) / 'linear_imu.csv'
            angular_path = Path(self.calibration_data_dir) / 'angular_imu.csv'
            
            if linear_path.exists():
                linear_data = np.genfromtxt(linear_path, delimiter=',', skip_header=1)
                self.accel_bias = np.mean(linear_data, axis=0)
                self.accel_std = np.std(linear_data, axis=0)
                if self.logger:
                    self.logger.info(
                        f'Loaded linear acceleration calibration: '
                        f'bias={self.accel_bias}, std={self.accel_std}'
                    )
            
            if angular_path.exists():
                angular_data = np.genfromtxt(angular_path, delimiter=',', skip_header=1)
                self.gyro_bias = np.mean(angular_data, axis=0)
                self.gyro_std = np.std(angular_data, axis=0)
                if self.logger:
                    self.logger.info(
                        f'Loaded angular velocity calibration: '
                        f'bias={self.gyro_bias}, std={self.gyro_std}'
                    )
                    
        except Exception as e:
            if self.logger:
                self.logger.warning(f'Failed to load calibration data: {e}')
    
    def remove_bias(self, 
                   accel: np.ndarray, 
                   gyro: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Remove bias (mean offset) from sensor readings.
        
        Args:
            accel: Linear acceleration [x, y, z] in m/s²
            gyro: Angular velocity [x, y, z] in rad/s
            
        Returns:
            Tuple of bias-corrected (accel, gyro) arrays
        """
        accel_corrected = accel - self.accel_bias
        gyro_corrected = gyro - self.gyro_bias
        return accel_corrected, gyro_corrected
    
    def low_pass_filter(self, 
                       accel: np.ndarray, 
                       gyro: np.ndarray,
                       alpha: float = 0.7) -> Tuple[np.ndarray, np.ndarray]:
        """Apply exponential moving average low-pass filter.
        
        Reduces high-frequency noise while preserving signal integrity.
        
        Args:
            accel: Linear acceleration [x, y, z]
            gyro: Angular velocity [x, y, z]
            alpha: Smoothing factor (0.0-1.0). Higher = more filtering.
            
        Returns:
            Filtered (accel, gyro) arrays
        """
        # Buffer-based simple moving average
        self.accel_buffer.append(accel)
        self.gyro_buffer.append(gyro)
        
        if len(self.accel_buffer) > 0:
            accel_filtered = np.mean(list(self.accel_buffer), axis=0)
            gyro_filtered = np.mean(list(self.gyro_buffer), axis=0)
        else:
            accel_filtered = accel
            gyro_filtered = gyro
            
        return accel_filtered, gyro_filtered
    
    def detect_outliers(self, 
                       accel: np.ndarray, 
                       gyro: np.ndarray,
                       accel_threshold: float = 3.0,
                       gyro_threshold: float = 3.0) -> bool:
        """Detect outliers using z-score method.
        
        Args:
            accel: Linear acceleration [x, y, z]
            gyro: Angular velocity [x, y, z]
            accel_threshold: Z-score threshold for acceleration (sigma)
            gyro_threshold: Z-score threshold for angular velocity (sigma)
            
        Returns:
            True if data is an outlier, False otherwise
        """
        # Avoid division by zero
        accel_std = np.maximum(self.accel_std, 1e-6)
        gyro_std = np.maximum(self.gyro_std, 1e-6)
        
        # Z-score for each component
        accel_zscore = np.abs((accel - self.accel_bias) / accel_std)
        gyro_zscore = np.abs((gyro - self.gyro_bias) / gyro_std)
        
        # Outlier if any component exceeds threshold
        is_outlier = (np.any(accel_zscore > accel_threshold) or 
                     np.any(gyro_zscore > gyro_threshold))
        
        return is_outlier
    
    def compute_covariance_matrices(self) -> Tuple[list, list]:
        """Compute covariance matrices from calibration statistics.
        
        Returns:
            Tuple of (accel_covariance, gyro_covariance) as 9-element lists
        """
        # Use standard deviation squared for covariance (diagonal matrix)
        accel_var = np.square(self.accel_std)
        gyro_var = np.square(self.gyro_std)
        
        # Flatten to 9-element ROS covariance format (row-major)
        accel_cov = [
            accel_var[0], 0.0, 0.0,
            0.0, accel_var[1], 0.0,
            0.0, 0.0, accel_var[2]
        ]
        
        gyro_cov = [
            gyro_var[0], 0.0, 0.0,
            0.0, gyro_var[1], 0.0,
            0.0, 0.0, gyro_var[2]
        ]
        
        return accel_cov, gyro_cov
    
    def preprocess(self,
                  accel: np.ndarray,
                  gyro: np.ndarray,
                  remove_outliers: bool = True,
                  apply_filter: bool = True,
                  outlier_threshold: float = 3.0) -> Tuple[np.ndarray, np.ndarray, bool]:
        """Full preprocessing pipeline.
        
        Args:
            accel: Linear acceleration [x, y, z] in m/s²
            gyro: Angular velocity [x, y, z] in rad/s
            remove_outliers: Whether to detect and flag outliers
            apply_filter: Whether to apply low-pass filter
            outlier_threshold: Z-score threshold for outlier detection
            
        Returns:
            Tuple of (processed_accel, processed_gyro, is_valid)
            is_valid=False indicates data should be discarded/ignored
        """
        is_valid = True
        
        # Step 1: Outlier detection (before any processing)
        if remove_outliers:
            is_valid = not self.detect_outliers(accel, gyro, outlier_threshold, outlier_threshold)
            if not is_valid:
                return accel, gyro, False
        
        # Step 2: Bias removal (calibration)
        accel, gyro = self.remove_bias(accel, gyro)
        
        # Step 3: Low-pass filtering (noise reduction)
        if apply_filter:
            accel, gyro = self.low_pass_filter(accel, gyro)
        
        return accel, gyro, is_valid
    
    def get_calibration_stats(self) -> Dict:
        """Return calibration statistics for logging/debugging."""
        return {
            'accel_bias': self.accel_bias.tolist(),
            'accel_std': self.accel_std.tolist(),
            'gyro_bias': self.gyro_bias.tolist(),
            'gyro_std': self.gyro_std.tolist(),
        }
