import rclpy
from rclpy.node import Node


class LinearActuator(Node):
    def __init__(self):
        super().__init__('LinearActuator')
