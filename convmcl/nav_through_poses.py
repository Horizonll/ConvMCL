import rclpy

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)


def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()
    if not navigator.getDockedStatus():
        navigator.info("Docking before intialising pose")
        navigator.dock()
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([-7.0, 8.0], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped([-5.0, -6.0], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([-2.0, 1.0], TurtleBot4Directions.SOUTH))
    navigator.undock()
    navigator.startFollowWaypoints(goal_pose)
    navigator.dock()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
