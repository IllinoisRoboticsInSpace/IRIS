#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class CytronLinearActuatorNode(Node):
    def __init__(self):
        super().__init__('cytron_lin_actr')
        self.port = '/dev/ttyACM0'
        
        # Serial port configuration
        self.serial_port = self.declare_parameter('serial_port', self.port).value
        self.baud_rate = self.declare_parameter('baud_rate', 9600).value
        
        # Motor configuration
        self.max_pwm = 255 
        self.current_speed = 0
        
        # Initialize serial connection
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            raise
        
        # Create subscribers for the trigger inputs
        self.contract_sub = self.create_subscription(
            Float32,
            '/gamepad/lin_actr_contract',
            self.contract_callback,
            10
        )
        
        self.expand_sub = self.create_subscription(
            Float32,
            '/gamepad/lin_actr_expand',
            self.expand_callback,
            10
        )
        
        # Timer for sending commands at a fixed rate
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Cytron Linear Actuator Node initialized')
    
    def contract_callback(self, msg):
        # Convert trigger value (0.0 to 1.0) to negative PWM value (-255 to 0)
        if msg.data > 0.1:  # Apply a small deadzone
            self.current_speed = -int(msg.data * self.max_pwm / 2)  # Half speed as requested
            self.get_logger().info(f"Contract command received: {msg.data}, setting speed to {self.current_speed}")
    
    def expand_callback(self, msg):
        # Convert trigger value (0.0 to 1.0) to positive PWM value (0 to 255)
        if msg.data > 0.1:  # Apply a small deadzone
            self.current_speed = int(msg.data * self.max_pwm / 2)  # Half speed as requested
            self.get_logger().info(f"Expand command received: {msg.data}, setting speed to {self.current_speed}")
    
    def timer_callback(self):
        # Send the current speed command to the Arduino
        command = f"M{self.current_speed}\n"
        try:
            self.serial.write(command.encode())
            self.get_logger().debug(f"Sent command: {command.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
    
    def destroy_node(self):
        # Stop the motor before shutting down
        try:
            self.serial.write(b"M0\n")
            self.serial.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = CytronLinearActuatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()