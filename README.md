Перед запуском добавить дополнительные пакеты: 

## twist_mux
```bash
sudo apt install ros-jazzy-twist-mux
```

или клонировать репозиторий:
```bash
git clone https://github.com/YJ0528/twist_mux.git
```
после чего: 
```bash
	colcon build --symlink-install --packages-select twist_mux
```

## twist_stamper
клонировать репозиторий:
```bash
git clone https://github.com/joshnewans/twist_stamper.git
```
после чего:
```bash
colcon build --symlink-install --packages-select twist_stamper
```

## запуск проекта
cимуляция + rviz:
```bash
ros2 launch edubot_gazebo sim.launch.py
```

teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=True
```

tf-преобразования:
```bash
ros2 run rqt_tf_tree rqt_tf_tree
```
