import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

import numpy as np
from scipy.spatial.transform import Rotation as R

class RsIMU(Node):
    def __init__(self):
        super().__init__('RS_IMU')

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.update)

        self.imu_sub = self.create_subscription(Imu, '/camera/imu', self.imu_callback, 10)

        self.orientation = R.from_rotvec([0, 0, 0])

        self.pos = np.zeros(3)
        self.vel = np.zeros(3)

        self.gyro = np.array([0, 0, 0])
        self.accel = np.array([0, 0, 0])

        self.i = 0
        self.j = 0
        self.totalR = np.zeros(3)
        self.totalA = np.zeros(3)

    def imu_callback(self, msg):
        ang_vel = msg.angular_velocity
        lin_accel = msg.linear_acceleration
        self.gyro = np.array([ang_vel.x, ang_vel.y, ang_vel.z])
        self.accel = np.array([lin_accel.x, lin_accel.y, lin_accel.z])
        self.totalR += self.gyro
        self.i+=1
        self.get_logger().info("AvgR: " + str(self.totalR / self.i))
        # self.get_logger().info("Accel: " + str(self.accel))
        # if (np.sqrt(np.array([n**2 for n in self.gyro]).sum()) > 3):
            # self.get_logger().info("Outlier: " + str(self.gyro))

    def update(self):
        delta = R.from_rotvec(self.gyro * self.timer_period)
        self.orientation = self.orientation * delta
        self.totalA += self.orientation.as_rotvec()
        self.j += 1
        # self.get_logger().info("AvgA: " + str(self.orientation.as_rotvec()))

def main(args=None):
    rclpy.init(args=args)
    
    rs_imu = RsIMU()
    rclpy.spin(rs_imu)
    
    rs_imu.destroy_node()
    rclpy.shutdown()