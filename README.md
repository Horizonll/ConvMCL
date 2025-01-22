# ConvMCL

安装依赖

```bash
rosdep install --from-path src -yi --rosdistro humble
```

编译

```bash
colcon build --symlink-install
```

启动仿真

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze
```

记录数据

```bash
ros2 bag record /scan /tf /tf_static /odom /sim_ground_truth_pose
```

设置初始位姿

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {
    frame_id: 'map',
    stamp: {sec: 0, nanosec: 0}
  },
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  }
}"
```

或

```bash
ros2 run convmcl init
```

数据回放

```bash
ros2 bag play rosbag*
```

启动定位

```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=maze.yaml
```
