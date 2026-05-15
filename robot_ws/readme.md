## Build

```bash
# clean
cd ~/dev/robot_ws
rm -rf build/ install/ log/

# build
colcon build --symlink-install
```

## Launch

```bash
# source  workspaces (alias)
start_ros

# robot launch file (alias)
robot_launch

# run microcontroller micro ros agent 
start_microros

# display rviz
ros2 launch bee_mobile display.launch.py
```

## Nav2

```bash
# save map
ros2 run nav2_map_server map_saver_cli -f ~/dev/robot_ws/src/bee_mobile/resource/my_map
```

## Plotjuggler

```bash
ros2 run plotjuggler plotjuggler
```