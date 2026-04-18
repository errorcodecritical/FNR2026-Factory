#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import serial
import time


class MinIMU9Node(Node):
    """ROS2 node for publishing MinIMU-9 v5 sensor data."""
    
    def __init__(self):
        super().__init__('minimu9_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM_ARDUINO')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Create publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        # Conversion factors for MinIMU-9 v5
        # LSM6DS33 Accelerometer: ±2g scale
        self.accel_scale = 9.80665 / 16384.0  # Convert to m/s²
        
        # LSM6DS33 Gyroscope: ±245 dps scale
        self.gyro_scale = (1.0 / 114.29) * (3.14159265359 / 180.0)  # Convert to rad/s
        
        # LIS3MDL Magnetometer: ±4 gauss scale
        self.mag_scale = 1.0 / 6842.0  # Convert to gauss
        
        # Initialize serial connection
        self.serial_conn = None
        self.connect_serial()
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info(f'MinIMU-9 Node started on {self.serial_port}')
    
    def connect_serial(self):
        """Establish serial connection with retries."""
        max_retries = 5
        retry_count = 0
        
        while retry_count < max_retries and self.serial_conn is None:
            try:
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=1.0
                )
                self.get_logger().info(f'Connected to {self.serial_port}')
                
                # Clear initial buffer and wait for valid data
                time.sleep(2)
                self.serial_conn.reset_input_buffer()
                
            except serial.SerialException as e:
                retry_count += 1
                self.get_logger().warning(
                    f'Failed to connect to {self.serial_port}: {e}. '
                    f'Retry {retry_count}/{max_retries}'
                )
                time.sleep(2)
        
        if self.serial_conn is None:
            self.get_logger().error(f'Could not connect to {self.serial_port}')
    
    def parse_serial_data(self, line):
        """Parse CSV data from serial port.
        
        Format: TIME,AX,AY,AZ,GX,GY,GZ,MX,MY,MZ
        """
        try:
            # Strip whitespace and decode
            line = line.strip().decode('utf-8')
            
            # Skip header or non-data lines
            if line.startswith('TIME') or line.startswith('Pololu') or \
               line.startswith('Initializing') or line.startswith('Format'):
                return None
            
            # Split CSV
            values = line.split(',')
            
            if len(values) != 10:
                return None
            
            # Parse values
            data = {
                'timestamp': int(values[0]),
                'accel_x': int(values[1]),
                'accel_y': int(values[2]),
                'accel_z': int(values[3]),
                'gyro_x': int(values[4]),
                'gyro_y': int(values[5]),
                'gyro_z': int(values[6]),
                'mag_x': int(values[7]),
                'mag_y': int(values[8]),
                'mag_z': int(values[9])
            }
            
            return data
            
        except (ValueError, UnicodeDecodeError) as e:
            self.get_logger().debug(f'Parse error: {e}')
            return None
    
    def timer_callback(self):
        """Read and publish sensor data."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().warning('Serial connection lost. Attempting reconnect...')
            self.connect_serial()
            return
        
        try:
            # Read line from serial
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline()
                data = self.parse_serial_data(line)
                
                if data is not None:
                    self.publish_imu_data(data)
                    self.publish_mag_data(data)
                    
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
            self.serial_conn = None
    
    def publish_imu_data(self, data):
        """Publish IMU message (accel + gyro)."""
        msg = Imu()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Linear acceleration (m/s²)
        msg.linear_acceleration.x = data['accel_x'] * self.accel_scale
        msg.linear_acceleration.y = data['accel_y'] * self.accel_scale
        msg.linear_acceleration.z = data['accel_z'] * self.accel_scale
        
        # Angular velocity (rad/s)
        msg.angular_velocity.x = data['gyro_x'] * self.gyro_scale
        msg.angular_velocity.y = data['gyro_y'] * self.gyro_scale
        msg.angular_velocity.z = data['gyro_z'] * self.gyro_scale
        
        # Orientation not provided by raw sensor
        msg.orientation_covariance[0] = -1.0
        
        # Covariance matrices (simplified - you may want to calibrate these)
        msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        self.imu_pub.publish(msg)
    
    def publish_mag_data(self, data):
        """Publish magnetometer message."""
        msg = MagneticField()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Magnetic field (Tesla)
        # Convert gauss to Tesla: 1 gauss = 1e-4 Tesla
        msg.magnetic_field.x = data['mag_x'] * self.mag_scale * 1e-4
        msg.magnetic_field.y = data['mag_y'] * self.mag_scale * 1e-4
        msg.magnetic_field.z = data['mag_z'] * self.mag_scale * 1e-4
        
        # Covariance matrix (simplified)
        msg.magnetic_field_covariance = [
            0.0001, 0.0, 0.0,
            0.0, 0.0001, 0.0,
            0.0, 0.0, 0.0001
        ]
        
        self.mag_pub.publish(msg)
    
    def destroy_node(self):
        """Clean up on shutdown."""
        if self.serial_conn is not None and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MinIMU9Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()