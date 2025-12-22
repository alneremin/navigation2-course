
# Аккерман - Гамберов Тимур и Мызина Александра

---

### Команды для запуска

```bash
ros2 launch f1_gazebo f1_gazebo.launch.py
```
```bash
ros2 launch f1_control control.launch.py
ros2 launch f1_navigation localization.launch.py
```
```bash
ros2 launch f1_navigation navigation.launch.py
```
```bash
ros2 run rviz2 rviz2 --ros-args -r /tf:=/f1/tf -r /tf_static:=/f1/tf_static
```
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/f1/ackermann_steering_controller/reference -p stamped:=true
```

---




