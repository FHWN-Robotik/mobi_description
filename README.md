# mobi_description

Test:\
Kopiere den **mobi_description** Ordner in den **src** Ordner. (ros2_ws/src) \
Führe die folgenden Zeilen im Terminal in deinem **WORKSPAC**E Ordner aus:

```shell
cd ~/ros2_ws
colcon build --packages-select mobi_description
source install/setup.bash
ros2 launch mobi_description gazebo.launch.py
```
