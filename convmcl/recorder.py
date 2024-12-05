import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import csv
import os
from datetime import datetime


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
        self.pose_list = []

    def sim_ground_truth_pose_callback(self, msg):
        self.sim_pose = msg.pose.pose

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg.pose.pose
        self.record()

    def record(self):
        if self.sim_pose and self.amcl_pose:
            self.pose_list.append(
                (
                    self.sim_pose.position.x,
                    self.sim_pose.position.y,
                    self.amcl_pose.position.x,
                    self.amcl_pose.position.y,
                )
            )

    def save_to_csv(self, directory):
        os.makedirs(directory, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}.csv"
        filepath = os.path.join(directory, filename)
        with open(filepath, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["real_x", "real_y", "amcl_x", "amcl_y"])
            writer.writerows(self.pose_list)
        print(f"Data saved to {filepath}")


def main():
    rclpy.init()
    data_recorder = DataRecorder()
    try:
        print("Recording data...")
        rclpy.spin(data_recorder)
    except KeyboardInterrupt:
        pass
    data_recorder.save_to_csv("/home/hrz/turtlebot4_ws/src/data")
    data_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
