"""
@file udp_server.py
@author Rushil Shah (email=rushils4@illinois.edu)
@maintainer Rushil Shah (email=shahrushil520@gmail.com)
@date November 17th, 2024

@brief Python-Based UDP Server for Jetson Orin Nano to 
Receive Game Controller Data from Laptop

@description Receive data from laptop client, decode message w/ 
Protobuf, publish to gamepad topics for arduino_comms_node.
"""

import rclpy
from rclpy.node import Node
import socket

class PythonUDP_Server(Node):

    sock : socket

    def __init__(self):
        super().__init__("python_udp_server")
        self.sock = socket.socket(
            socket.AF_INET, 
            socket.SOCK_DGRAM
        )
        self.sock.bind(('0.0.0.0', 8080))
        self.get_logger().info("Python UDP Server listening on port 8080")

        while True:
            # Record the data and what IP address it's from with a buffer size 1024 bytes.
            data, address = self.sock.recvfrom(1024)

            # Log Data Before Decoding
            self.get_logger().info(f"Received: {data} from {address}")

            # Decode the Protobuf Message

def main(args=None):
    rclpy.init(args=args)
    server : PythonUDP_Server = PythonUDP_Server()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()