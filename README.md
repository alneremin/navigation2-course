
# Аккерман - Гамберов Тимур и Мызина Александра

---

### Команды для запуска

1. ```bash
ros2 launch f1_gazebo f1_gazebo.launch.py
```
3. ros2 launch f1_control control.launch.py
4. ros2 launch f1_navigation localization.launch.py
5. ros2 launch f1_navigation navigation.launch.py
6. ros2 run rviz2 rviz2 --ros-args -r /tf:=/f1/tf -r /tf_static:=/f1/tf_static
7. ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/f1/ackermann_steering_controller/reference -p stamped:=true

---




