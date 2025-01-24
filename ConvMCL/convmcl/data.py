import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Data(Node):
    def __init__(self):
        super().__init__("data")
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.scan = None

    def odom_callback(self, msg):
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z
        print(f"Linear: {self.linear_vel}, Angular: {self.angular_vel}")

    def scan_callback(self, msg):
        self.scan = msg.ranges
        print(self.scan[:5])


def main(args=None):
    rclpy.init(args=args)
    data = Data()
    rclpy.spin(data)
    data.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
