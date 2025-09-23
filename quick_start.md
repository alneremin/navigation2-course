# Quick Start

## Оглавление
- [Установка необходимых пакетов](#установка-необходимых-пакетов)
  - [ROS 2 Jazzy Jalisco](#ros-2-jazzy-jalisco)
  - [Gazebo Harmonic](#gazebo-harmonic)
  - [Nav2](#nav2)
- [Работа со средами](#работа-со-средами)
  - [ROS 2](#ros2)
  - [Gazebo Harmonic](#gazebo-harmonic-1)
  - [RViz (Robot Visualization)](#rviz-robot-visualization)

---

## Установка необходимых пакетов

### ROS 2 Jazzy Jalisco
[Инструкция по установке](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

Проверяем работу ROS 2 на примере подписчика и издателя:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker

# переходим в новый терминал
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

---

### Gazebo Harmonic
[Инструкция по установке](https://gazebosim.org/docs/harmonic/install_ubuntu/)

Проверяем работу Gazebo Harmonic на примере мира с примитивами:

```bash
gz sim shapes.sdf
```

---

### Nav2
[Инструкция по установке](https://docs.nav2.org/getting_started/index.html#installation)

Проверяем работу Nav2 на примере навигации TurtleBot3:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

---

## Работа со средами

### ROS2

С основными командами можно ознакомиться в [Cheat Sheet](https://www.theconstruct.ai/wp-content/uploads/2021/10/ROS2-Command-Cheat-Sheets-updated.pdf)

Основные команды, которыми будем пользоваться:

#### Создание и сборка пакетов
- Создание узла (node) на C++:  
  ```bash
  ros2 pkg create --build-type ament_cmake <package-name>
  ```
- Сборка всех проектов в воркспейсе:  
  ```bash
  colcon build
  ```
- Сборка отдельных пакетов:  
  ```bash
  colcon build --packages-select <package-name>
  ```

#### Запуск узлов
- Запуск узла (node) с параметрами:  
  ```bash
  ros2 run <package-name> <node> --ros-args -p <param1>:=<value>
  ```
- Запуск узла с переназначением (remap) топика/сервиса/действия (action):  
  ```bash
  ros2 run <package-name> <node> --ros-args -r <old-topic-name>:=<new-topic-name>
  ```
- Запуск launch-файла с аргументами:  
  ```bash
  ros2 launch <package-name> <launch-file> <param1>:=<value1> <param2>:=<value2>
  ```

#### Отладка работающей системы
- Информация об узле:  
  ```bash
  ros2 node info /<node-name>
  ```
- Список топиков:  
  ```bash
  ros2 topic list
  ```
- Информация об отдельном топике:  
  ```bash
  ros2 topic info /<topic-name>
  ```
- Прослушка топика:  
  ```bash
  ros2 topic echo /<topic-name>
  ```
- Публикация в топик:  
  ```bash
  ros2 topic pub /<topic-name> <package-name>/msg/<MsgType> '{<JSON-message-payload>}'
  ```
- Частота публикации:  
  ```bash
  ros2 topic hz /<topic-name>
  ```
- Список сервисов:  
  ```bash
  ros2 service list
  ```
- Информация об отдельном сервисе:  
  ```bash
  ros2 service info /<service-name>
  ```
- Вызов сервиса:  
  ```bash
  ros2 service call /<service-name> <package-name>/srv/<SrvType> '{<JSON-service-payload>}'
  ```
- Список параметров:  
  ```bash
  ros2 param list
  ```
- Информация об отдельном параметре:  
  ```bash
  ros2 param get /<node-name> <parameter-name>
  ```
- Установка значения отдельного параметра:  
  ```bash
  ros2 param set /<node-name> <parameter-name> <value>
  ```

---

### Gazebo Harmonic

Работа со средой Gazebo Harmonic похожа структурно на работу с ROS. Gazebo Harmonic имеет, как и ROS, **топики** (topics) и **сервисы** (services).

Основные команды, которыми будем пользоваться:

- Запуск мира SDF:  
  ```bash
  gz sim <sdf-file-name>
  ```
- Список топиков:  
  ```bash
  gz topic -l
  ```
- Прослушка топика:  
  ```bash
  gz topic -e -t <topic-name>
  ```
- Список сервисов:  
  ```bash
  gz service -l
  ```
- Вызов сервиса:  
  ```bash
  gz service -s <service-name> --reqtype gz.msgs.<request-type> --reptype gz.msgs.<response-type> --timeout <timeout-value> --req <data>
  ```

В самом Gazebo также есть возможность управления данными через компоненты. Рассмотрим основные компоненты для работы. 
- Grid Config - позволяет задавать параметры отображаемой клетки
- Image Display - дает возможность получить изображение с камер в Gazebo
- Publisher - позволяет настроить публикацию в топики Gazebo
- Topic Echo - позволяет читать топики Gazebo
- Topic Viewer - позволяет просмотреть структуру топиков
- Teleop - позволяет управлять роботом посредством телеоперации
- Visualize Lidar - позволяет отобразить лучи дальномера в Gazebo

### RViz (Robot Visualization)

Для визуализации данных робота (структура робота, показания датчиков, текущее положение, траектория) и окружения (карта препятствий, карта стоимости) в ROS используется улитита RViz. Эта утилита устанавливается вместе с ROS при установке полной версии (desktop). Если утилита не установлена, необходимо ввести команду установки через apt.

```bash
sudo apt install ros-jazzy-rviz2
```

Запуск RViz может быть произведен через командную строку.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2
```

RViz функционально состоит из верхней панели инструментов и боковых панелей для настройки отображаемых данных (Displays) и настройки положения камеры в RViz (Views).

Верхняя панель содержит следующий инструментарий:
- Расчет длины отрезка (Measure)
- Отправка начальной позиции на робота (2D Pose Estimate)
- Отправка целевой позиции на робота (2D Goal Pose)
- Отправка точки в пространстве (Publish Point)
- Пользовательские плагины

Панель отображаемых данных (Displays) может иметь различные конфигурации в зависимости от поставленной задачи. В задачи навигации конфигурация традиционно включает в себя:
- Модель робота (Robot Model)
- Данные с датчиков (LaserScan, Camera, )
- Карту (Map)
- Траектории движения робота (Path)
- Группы (группировка отображаемых данных)
- Пользовательские плагины

Настройка положения камеры применяется реже, поскольку управление камерой может выполняться мышью. Настройка включает в себя выбор типа камеры и параметров для этого вида:
- FPS (First-Person Shooter) позволяет перемещаться по сцене так, как будто вы наблюдаете из некоторой точки
- FrameAligned позволяет двигаться камере вдоль заданной плоскости
- Орбитальная камера (Orbit) вращается вокруг фокусной точки, отмеченной желтым пятном в пространстве
- ThirdPersonFollower поддерживает постоянный угол обзора по отношению к целевому кадру
- TopDownOrtho дает возможность зафиксировать ось перемещения Z и оси вращения X и Y
- XY Orbit - это орбитальная камера с точкой фокусировки, ограниченной плоскостью XY