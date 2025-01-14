import time
import rclpy
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)
from rclpy.node import Node


class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")
        self.navigator = TurtleBot4Navigator()
        if not self.navigator.getDockedStatus():
            self.navigator.info("Docking before initializing pose")
            self.navigator.dock()
        time.sleep(2)
        initial_pose = self.navigator.getPoseStamped(
            [0.0, 0.0], TurtleBot4Directions.NORTH
        )
        self.navigator.setInitialPose(initial_pose)
        # self.navigator.undock()


def main():
    rclpy.init()
    navigator = Navigator()
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
