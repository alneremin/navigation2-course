
# Robot description

## Оглавление
- [Создание проекта](#создание-проекта)
- [Создание базы робота](#создание-базы-робота)
- [Настройка проекта](#настройка-проекта)
  - [Подготовка CMakeLists.txt](#подготовка-cmakeliststxt)
  - [Настройка файла запуска (launch)](#настройка-файла-запуска-launch)
- [Запуск!](#запуск)

## Создание проекта

Создаем в домашней директории (~) рабочую среду (workspace), в которой будем работать
```
mkdir nav_ws
cd nav_ws
colcon build
```

Поскольку у нас пустая рабочая среда, то список пакетов будет пустым
```
colcon info
```

Создадим пакет описания робота
```
mkdir src
cd src
ros2 pkg create --build-type ament_cmake bmx_description
```

Получим следующую структуру проекта:

```
bmx_description/
├── include/
│   └── bmx_description
├── src
├── CMakeLists.txt
└── package.xml
```

Удалим директории для хранения исходного кода и добавим папки **urdf** для хранения urdf/xacro файлов, **meshes** для хранения моделей и **launch** для хранения файлов запуска. В каждой папке создадим директории для хранения отдельных компонент робота. Получим следующую структуру: 

```
bmx_description/
├── launch/
├── meshes/
│   ├── base
│   ├── sensors
│   └── wheels
├── urdf/
│   ├── robots
│   └── components/
│       ├── base
│       ├── plugins
│       ├── sensors
│       └── wheels
├── CMakeLists.txt
└── package.xml
```

## Создание базы робота

Перейдем к созданию базы робота или шасси, на которую будем крепить остальные элементы. Создадим файл **base.urdf.xacro** в директории *urdf/components/base/base.urdf.xacro*. Оформим XML как параметризированный компонент, или макрос.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:macro name="base">

    </xacro:macro>
</robot>
```

Добавим корневое соединение base_footprint.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:macro name="base">
        <link name="base_footprint"/>
    </xacro:macro>
</robot>
```

Добавим шасси в виде соединения base_link, указав визуальную и коллизионную часть в виде правильного паралеллограмма 0.5x0.1x0.3.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:macro name="base">
        
        ...

        <link name="base_link">
            <visual>
                <origin xyz="0 0 0" rpy="0.0 0.0 0.0" />
                <geometry>
                    <box size="0.5 0.1 0.3"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0.0 0.0 0.0" />
                <geometry>
                    <box size="0.5 0.1 0.3"/>
                </geometry>
            </collision>
            <inertial>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <mass value="5.0"/>
                <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
            </inertial>
        </link>
    </xacro:macro>
</robot>
```

Свяжем соединения, добавив сустав base_link_joint.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:macro name="base">
        ...
        <joint name="base_link_joint" type="fixed">
            <origin xyz="0.0 0.0 0.15" rpy="0.0 0.0 0.0"/>
            <parent link="base_footprint"/>
            <child link="base_link"/>
        </joint>
    </xacro:macro>
</robot>
```

Создадим модель робота. Для этого в папке *urdf/robots/bmx.urdf.xacro* создадим файл **bmx.urdf.xacro**. В файле укажем в виде аргументов название робота, пространство имен, а также подключим макрос базы, который мы написали.

```xml
<?xml version="1.0"?>
<robot name="bmx" xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:arg name="robot_name" default="bmx"/>
    <xacro:arg name="robot_namespace" default="$(arg robot_name)"/>

    <xacro:include filename="$(find bmx_description)/urdf/components/base/base.urdf.xacro" />

    <xacro:base />
</robot>
```

## Настройка проекта

### Подготовка CMakeLists.txt

Наша база робота готова, теперь необходимо запустить нашу модель. Прежде всего настроим CMakeLists.txt, чтобы после сборки проекта файлы были доступны при запуске launch-скриптов. Для этого перейдем в CMakeLists.txt и добавим в конец файла следующие строки:

```cmake
cmake_minimum_required(VERSION 3.8)
project(bmx_description)

...

install(
  DIRECTORY launch meshes urdf 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

```

### Настройка файла запуска (launch)

Для запуска проекта необходимо настроить launch-файл. Для этого в папке */launch* создадим файл **description.launch.py**. Этот файл является скриптом на Python. Содержание этого файла регламентировано. При запуске файла ROS ищет функцию generate_launch_description(), возвращающую объекта класса LaunchDescription. Таким образом изначально файл выглядит следующим образом:

```python

from launch import LaunchDescription

def generate_launch_description():

	# Create the launch description and populate
	ld = LaunchDescription()

	return ld
```

Создадим переменную robot_xacro_path, в которой будем хранить путь до xacro-файла описания робота. Укажем путь до файла, используя встроенную функцию os.path.join языка и функцию get_package_share_directory из ROS2. Предварительно подключим библиотеки для использования этих функций. Получим

```python

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():
    robot_xacro_path = os.path.join(get_package_share_directory('bmx_description'), 'urdf/robots', 'bmx.urdf.xacro')

	...
```

Добавим аргумент в launch-файл, в нашем случае это флаг **use_sim_time** и пространство имен **namespace**, которое мы передадим в модель робота. Для объявления аргументов используем объект класса LaunchConfiguration, предварительно его импортировав.

```python

...
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
	...
    # Create the launch configuration variables
	namespace = LaunchConfiguration('namespace')
	use_sim_time = LaunchConfiguration('use_sim_time')
    ...
```

Для добавления значений аргументов по умолчанию и их описания создадим объекты класса DeclareLaunchArgument, также его предварительно подключив в файле.

```python

...
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
	...
	# Declare the launch arguments
	declare_namespace_cmd = DeclareLaunchArgument(
		name='namespace',
		default_value='bmx',
		description='Top-level namespace'
	)

	declare_use_sim_time_cmd = DeclareLaunchArgument(
		name='use_sim_time',
		default_value='false',
		description='Use simulation (Gazebo) clock if true'
	)
    ...
```

Опишем узел (node), который будет публиковать описание робота и трансформации между соединениями робота. Для этого создадим объект класса Node (подключив класс в шапке файла) и зададим его параметры.

```python

...
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
	...
	# Declare the launch nodes
	robot_state_publisher_cmd = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='screen',
		namespace=namespace,
		parameters=[{'robot_description': ParameterValue(Command(['xacro ', robot_xacro_path, ' robot_namespace:=', namespace]), value_type=str)},
					{'use_sim_time': use_sim_time}],
		remappings=[('/tf','tf'),
					('/tf_static','tf_static')]
	)
    ...
```

После создания необходимых объектов их необходимо добавить в возвращаемый объект LaunchDescription. Для этого используется метод add_action().

```python

...

def generate_launch_description():
	...

	# Create the launch description and populate
	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(declare_namespace_cmd)

	# Add the actions to launch all of the nodes
	ld.add_action(robot_state_publisher_cmd)

	return ld
    ...
```

## Запуск!

Файл запуска готов. Выполняем сборку проекта: ```colcon build```
Запускаем launch-файл, указав пространство имен для терминала.
```bash
source ~/nav_ws/install/setup.bash
ros2 launch bmx_description description.launch.py
```

После запуска launch-файла робот не запустился в Gazebo, как и не запустился сам Gazebo. Все потому, что мы не указали файл запуска для Gazebo и файл запуска для спавна нашего робота в Gazebo. 

Мы можем проверить, правильно ли мы создали структуру робота и заполнили файлы. Для этого в новом терминале пропишем команду ```ros2 topic list```. Мы должны увидеть топики, публикующие URDF-описание робота (/bmx/robot_description), топики, публикующие трансформации между соединениями робота (/bmx/tf и /bmx/tf_static). Мы можем проверить наличие данных с помощью соответствующих команд.

```bash
# Построение дерева соединений (links) робота
sudo apt install ros-jazzy-rqt-tf-tree
ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=/bmx/tf -r /tf_static:=/bmx/tf_static

# Получение URDF-файла робота
ros2 topic echo /bmx/robot_description -f
```