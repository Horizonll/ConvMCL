import rclpy
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)
import time


def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()
    if not navigator.getDockedStatus():
        navigator.info("Docking before intialising pose")
        navigator.dock()
    initial_pose = navigator.getPoseStamped([9.1, -8.4], TurtleBot4Directions.EAST)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([8.2, 9.4], TurtleBot4Directions.WEST))
    # goal_pose.append(navigator.getPoseStamped([-5.0, -6.0], TurtleBot4Directions.EAST))
    # goal_pose.append(navigator.getPoseStamped([-2.0, 1.0], TurtleBot4Directions.SOUTH))
    navigator.undock()
    print("undocked")
    time.sleep(1)
    navigator.startFollowWaypoints(goal_pose)
    # navigator.dock()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
