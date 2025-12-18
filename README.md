# Homework 1 - Робот доставщик Яндекс с diff_drive_controller

## Структура пакетов:
- **ya_rover_description** 
    XACRO файлы с моделью робота и launch файлы для просмотра модели.
    Посмотреть модель в rviz:
    `ros2 launch ya_rover_description display_rviz.launch.py`
- **ya_rover_control**
    - yaml файл с контроллером и launch файл для запуска контроллера
    - Запустить контроллеры: 
    `ros2 launch ya_rover_control control.launch.py`
- **ya_rover_gazebo**
    - bridge для gazebo, файлы мира, в котором запускается симуляция и launch файлы для запуска симуляции
    - Запустить симуляцию:
    `ros2 launch ya_rover_gazebo final_gazebo.launch.py`


## Запуск симуляции с контроллерами и teleop
 (по очереди)
- `ros2 launch ya_rover_gazebo final_gazebo.launch.py`
- `ros2 launch ya_rover_control control.launch.py`
- `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=True`

- rqt_tf_tree:
 ` ros2 run rqt_tf_tree rqt_tf_tree`

# Homework 2 - Навигация для робота
- **Local planner**: DWB Controller
- **Global planner**: Smac Planner
## Запуск симуляции 
- `ros2 launch ya_rover_gazebo final_gazebo.launch.py` 
    - запуск gazebo
- `ros2 launch ya_rover_control control.launch.py` 
    - запуск котроллеров
- `ros2 launch ya_rover_navigation localization.launch.py`
    - запуск локализации
- `ros2 launch ya_rover_navigation navigation.launch.py`
    - запуск навигации
- `ros2 launch ya_rover_navigation display_rviz.launch.py`
    - отображение в rviz
- `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=True`
    - управление в teleop