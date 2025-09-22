# Robot in Gazebo

## Оглавление
- [Создание проекта](#создание-проекта)
- [Настройка виртуального мира](#настройка-виртуального-мира)
- [Настройка проекта](#настройка-проекта)
  - [Настройка файла запуска Gazebo](#настройка-файла-запуска-gazebo)
  - [Настройка CMakeLists.txt](#настройка-cmakeliststxt)
- [Запуск!](#запуск)

## Создание проекта

Создадим пакет для работы робота в Gazebo
```
cd ~/nav_ws/src
ros2 pkg create --build-type ament_cmake bmx_gazebo
```

Удалим директории для хранения исходного кода и добавим папки **worlds** для хранения файлов виртуальных миров Gazebo, **config** для хранения конфигураций и **launch** для хранения файлов запуска: 

```
bmx_description/
├── config/
├── launch/
├── worlds/
├── CMakeLists.txt
└── package.xml
```

## Настройка виртуального мира

Создадим пустой мир Gazebo с подстилающей плоскостью. Для этого будем придерживаться стандарта [SDF](http://sdformat.org/). Создадим пустой мир с помощью тега \<world\>.

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

  </world>
</sdf>

```

Опишем физику виртуального мира в теге \<physics\>:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

  </world>
</sdf>

```

Настроим сцену, для этого укажем тег \<scene\> и отключим тени в Gazebo.

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    ...
    <scene>
      <shadows>false</shadows>
    </scene>

  </world>
</sdf>

```

Настроим свет в Gazebo, для этого укажем тег \<light\> и зададим позицию источника света, параметры света, яркость, направление.

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    ...
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

  </world>
</sdf>

```

Наконец зазадим модель подстилающей поверхности, представляющей из себя плоскость с нормалью к оси Z и размером 100х100. Флаг \<static\> поднимем, чтобы плоскость быза зафиксирована в своей позиции. Поскольку позиции не задана явно, она равняется (0, 0, 0).

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    ...
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>

```

## Настройка проекта

### Настройка файла запуска Gazebo

Для запуска проекта необходимо настроить launch-файл. Для этого в папке */launch* создадим файл **gazebo.launch.py**. Зададим функцию generate_launch_description(), возвращающую объекта класса LaunchDescription следующим образом:

```python

from launch import LaunchDescription

def generate_launch_description():

	# Create the launch description and populate
	ld = LaunchDescription()

	return ld
```

Создадим переменную world_path, в которой будем хранить путь до world-файла Gazebo. В нашем случае это путь до файла *worlds/empty_world.world*.

```python

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription

def generate_launch_description():
    world_path = LaunchConfiguration('world_path')

	declare_world_path_cmd = DeclareLaunchArgument(
		name='world_path',
		default_value=os.path.join(get_package_share_directory('bmx_gazebo'), 'worlds', 'empty_world.world'),
		description='Specify world file'
	)
	...
```

Следующими укажем файлы запуска самого Gazebo. Для этого мы можем использовать ноды (nodes), но удобнее воспользоваться готовыми launch-файлами для запуска клиентской и серверной частей Gazebo. Launch-файлы будем запускать с помощью объекта IncludeLaunchDescription, указав путь до файла запуска.

```python

...
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    ...
    
    gzserver_cmd = IncludeLaunchDescription(
		os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
		launch_arguments={'gz_args': ['-r -s -v4 ', world_path],
						  'on_exit_shutdown': 'true'}.items()
	)

	gzclient_cmd = IncludeLaunchDescription(
		os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
		launch_arguments={'gz_args': '-g -v4 '}.items()
	)
	...
```


После создания необходимых объектов добавим их выволнение в объект LaunchDescription.

```python

...

def generate_launch_description():
	...

	# Create the launch description and populate
	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(declare_world_path_cmd)

	# Add the actions to launch all of the files and nodes
	ld.add_action(gzserver_cmd)
	ld.add_action(gzclient_cmd)

	return ld
    ...
```

### Настройка CMakeLists.txt

Запустим пустой мир в Gazebo Harmonic. Прежде всего настроим CMakeLists.txt, чтобы после сборки проекта были доступны launch-скрипты и файлы мира. Для этого перейдем в CMakeLists.txt и добавим в конец файла следующие строки:

```cmake
cmake_minimum_required(VERSION 3.8)
project(bmx_gazebo)

...

install(
  DIRECTORY launch worlds config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

```

## Запуск!

Файл запуска готов. Выполняем сборку проекта: ```colcon build```
Запускаем launch-файл, указав пространство имен для терминала.
```bash
source ~/nav_ws/install/setup.bash
ros2 launch bmx_gazebo gazebo.launch.py
```