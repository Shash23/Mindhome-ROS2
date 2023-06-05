import rclpy
from rclpy.node import Node
import time
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile

# call back libraries
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


class ControlClass(Node):

    def __init__(self, seconds_sleeping = 10):
        
        super().__init__('sub_node')
        self._seconds_sleeping = seconds_sleeping
        
        # publishers
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
        
        # create three call back groups
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()
        
        # subscriber for the odom topic
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10,  callback_group=self.group1)
        
        # uses for the /scan topic
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.group2)
        
        # timer object
        self.timer = self.create_timer(
            0.5, self.timer_callback, callback_group=self.group3)
        self.laser_msg = LaserScan()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    # call back for the odom subscriber
    def odom_callback(self, msg):
        self.get_logger().info("O callback")
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = self.euler_from_quaternion(orientation_list)

    # callback scanner for /scan subscriber
    def scan_callback(self, msg):
        self.get_logger().info("Scan CallBack")
        self.laser_msg = msg

    # lazer tag ranges
    def get_front_laser(self):
        return self.laser_msg.ranges[360]

    # yaw value
    def get_yaw(self):
        return self.yaw

    # quarterion for euler values
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to Euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
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

    # velocities to stop the robot
    def stop_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)

    # forward
    def move_straight(self):
        self.cmd.linear.x = 0.08
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)

    # rotate
    def rotate(self):
        self.cmd.angular.z = -0.2
        self.cmd.linear.x = 0.0
        self.vel_pub.publish(self.cmd)

        self.get_logger().info("Rotating for "+str(self._seconds_sleeping)+" seconds")
        
        for i in range(self._seconds_sleeping):
            self.get_logger().info("SLEEPING=="+str(i)+" seconds")
            time.sleep(1)

        self.stop_robot()

    # timer object
    def timer_callback(self):
        self.get_logger().info("Timer CallBack")
        try:
            self.get_logger().warning(">>>>>>>>>>>>>>RANGES Value=" +
                                      str(self.laser_msg.ranges[360]))
            if not self.laser_msg.ranges[360] < 0.5:
                self.get_logger().info("MOVE STRAIGHT")
                self.move_straight()
            else:
                self.get_logger().info("STOP ROTATE")
                self.stop_robot()
                self.rotate()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    control_node = ControlClass()
    # multithreading 
    executor = MultiThreadedExecutor(num_threads=4)
    
    executor.add_node(control_node)
    try:
        # Spin the executor
        executor.spin()
    finally:
        # Shutdown the executor
        executor.shutdown()
        control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()