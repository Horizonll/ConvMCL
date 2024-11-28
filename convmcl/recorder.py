from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math
import csv
import matplotlib.pyplot as plt
import datetime
import os


class TurtleBot4FirstNode(Node):
    def __init__(self):
        super().__init__("recorder")
        self.sim_ground_truth_pose_subscriber = self.create_subscription(
            Odometry,
            "/sim_ground_truth_pose",
            self.sim_ground_truth_pose_callback,
            qos_profile_sensor_data,
        )
        self.amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_pose_callback,
            qos_profile_sensor_data,
        )
        self.sim_pose = None
        self.amcl_pose = None
        self.errors = []
        self.time = datetime.datetime.now().strftime("%Y%m%d_%H:%M")
        self.data_directory = os.path.expanduser("~/data")

    def sim_ground_truth_pose_callback(self, msg):
        self.sim_pose = msg.pose.pose

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg.pose.pose
        self.calculate_error()

    def calculate_error(self):
        if self.sim_pose and self.amcl_pose:
            error_x = self.sim_pose.position.x - self.amcl_pose.position.x
            error_y = self.sim_pose.position.y - self.amcl_pose.position.y
            error = math.hypot(error_x, error_y)
            self.errors.append(error)
            print(f"Localization error: {100*error:.1f} cm")

    def display_errors(self):
        plt.plot(self.errors)
        plt.ylabel("Localization error (m)")
        plt.xlabel("Time Step")
        plt.title("Localization error over time")
        plt.show()

    def create_directory(self, subdir):
        directory = os.path.join(self.data_directory, subdir)
        if not os.path.exists(directory):
            os.makedirs(directory)
        return directory

    def save_data(self):
        directory = self.create_directory("csv")
        filename = os.path.join(directory, f"{self.time}.csv")
        try:
            with open(filename, mode="w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Error (m)"])
                writer.writerows([[error] for error in self.errors])
        except IOError as e:
            self.get_logger().error(f"Failed to save data: {e}")

    def save_svg(self):
        directory = self.create_directory("svg")
        filename = os.path.join(directory, f"{self.time}.svg")
        try:
            plt.plot(self.errors)
            plt.ylabel("Localization error (m)")
            plt.xlabel("Time Step")
            plt.title("Localization error over time")
            plt.savefig(filename)
        except IOError as e:
            self.get_logger().error(f"Failed to save SVG: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4FirstNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.display_errors()
    node.save_data()
    node.save_svg()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
