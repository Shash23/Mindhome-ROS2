import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.subscription_

        self.target_distance = 2.0  # Adjust this value to determine the desired distance to move forward
        self.target_angle = np.pi/2  # Adjust this value to determine the desired angle to rotate

        self.robot_orientation = None
        self.robot_position_x = None
        self.is_parallel = False
        self.is_rotated = False

    def odometry_callback(self, msg):
        # Extract robot orientation from odometry message
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        roll, pitch, yaw = self.euler_from_quaternion(quaternion)
        self.robot_orientation = yaw

        # Extract robot position from odometry message
        self.robot_position_x = msg.pose.pose.position.x

        if not self.is_parallel:
            # Move forward until the robot is in parallel with the opening
            if self.robot_position_x >= self.target_distance:
                self.is_parallel = True
                self.stop_robot()
        elif not self.is_rotated:
            # Rotate 90º in the direction of the opening
            if abs(self.robot_orientation - self.target_angle) <= 0.1:
                self.is_rotated = True
                self.stop_robot()
        else:
            # Move forward until the robot goes past the opening
            if self.robot_position_x <= self.target_distance:
                self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def euler_from_quaternion(self, quaternion):
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
