import rclpy
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import csv
import datetime
import os


class DataRecorder(Node):
    def __init__(self):
        super().__init__("data_recorder")
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
        self.sim_pose_list = []
        self.amcl_pose_list = []
        self.time = datetime.datetime.now().strftime("%Y%m%d_%H:%M")

    def sim_ground_truth_pose_callback(self, msg):
        self.sim_pose = msg.pose.pose

    def record(self):
        if self.sim_pose and self.amcl_pose:
            self.sim_pose_list.append(
                (self.sim_pose.position.x, self.sim_pose.position.y)
            )
            self.amcl_pose_list.append(
                (self.amcl_pose.position.x, self.amcl_pose.position.y)
            )

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg.pose.pose
        self.record()

    def save_to_csv(self, directory):
        filename = os.path.join(directory, f"{self.time}.csv")
        with open(filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["sim_pose_x", "sim_pose_y", "amcl_pose_x", "amcl_pose_y"])
            for sim_pose, amcl_pose in zip(self.sim_pose_list, self.amcl_pose_list):
                writer.writerow([sim_pose[0], sim_pose[1], amcl_pose[0], amcl_pose[1]])
        self.get_logger().info(f"Data saved to {filename}")


class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")
        self.navigator = TurtleBot4Navigator()
        if not self.navigator.getDockedStatus():
            self.navigator.info("Docking before initializing pose")
            self.navigator.dock()
        initial_pose = self.navigator.getPoseStamped(
            [9.1, -8.4], TurtleBot4Directions.EAST
        )
        self.navigator.setInitialPose(initial_pose)
        self.navigator.undock()


def main():
    rclpy.init()
    navigator = Navigator()
    data_recorder = DataRecorder()
    rclpy.spin(data_recorder)
    data_recorder.save_to_csv("data")
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
