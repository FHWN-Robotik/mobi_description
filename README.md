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

Folgende Packages werden benötigt:\
**joint_state_publisher_GUI**
```shell
sudo apt-get update
sudo apt install ros-humble-joint-state-publisher-gui
```
**xacro**
```shell
sudo apt-get update
sudo apt install ros-humble-xacro
```
**teleop_twist_keyboard**
```shell
sudo apt-get update
sudo apt install ros-humble-teleop-twist-keyboard
```
**Gazebo Classic**
```shell
sudo apt install ros-humble-gazebo-ros-pkgs
```
