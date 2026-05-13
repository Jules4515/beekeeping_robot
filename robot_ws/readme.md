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