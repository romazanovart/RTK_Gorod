# Решение «Движение по городу»

Основной код в репозитории перенесён из решения минского этапа 5–6 сентября 2026 года. Параметры оборудования, карта и калибровки соответствуют предоставленной папке. При переносе на другой робот нужно проверить устройства, направление вращения моторов и параметры движения.

## Состав решения

- `frob_sign_detection` открывает камеру, запускает YOLO и публикует изображения и обнаруженные знаки.
- `frob_city_mission` обрабатывает белую разметку с помощью OpenCV, выбирает действия по знакам, останавливается перед препятствиями по данным `/scan` и управляет состояниями миссии.
- `ros2_arduino_bridge` передаёт команды скорости и дискретных перемещений на Arduino.
- `frob_bringup` запускает мост, лидар, IMU, описание робота и одометрию. В поставленной конфигурации выбран драйвер `sllidar_c1_launch.py`.
- `frob_navigation` содержит карту и отдельные средства SLAM/Nav2. Они не запускаются автоматически узлом городской миссии: опубликованная миссия следует разметке камеры.

Дополнительно сохранён [исходный `move.py`](../examples/minsk/README.md) из корня минской папки: отдельный сценарий фиксированных перемещений через ROS-сервисы. Он не входит в автоматический запуск миссии.

## 1. Подготовка

Нужны ROS 2 Jazzy, Python 3.12, инструменты `colcon` и `rosdep`, подключённые камера, лидар и Arduino с прошивкой из `ros2/src/arduino2/arduino_code`. Базовая установка платформы описана в [руководстве Frob](MANUAL.md).

Клонируйте репозиторий в выбранный каталог и перейдите в него:

```bash
git clone https://github.com/romazanovart/RTK_Gorod.git
cd RTK_Gorod
source /opt/ros/jazzy/setup.bash
```

Веса `best.pt` уже включены в репозиторий. Проверить их контрольную сумму можно командой:

```bash
sha256sum ros2/src/ros2_ws/src/frob_sign_detection/models/best.pt
```

Ожидаемая сумма: `e609e19448bf5c6f3678012fa7611b8030bd0b8fcc0343ef134a72d9aca47ef9`.

Создайте окружение для детектора с доступом к системным ROS-пакетам:

```bash
python3 -m venv --system-site-packages ros2/venv-yolo
ros2/venv-yolo/bin/python -m pip install -r requirements-yolo.txt
```

В переданном окружении были установлены Ultralytics 8.4.140, PyTorch 2.14.0+cpu, OpenCV 5.0.0.93 и NumPy 2.5.2. `requirements-yolo.txt` фиксирует версию Ultralytics; остальные зависимости устанавливаются пакетным менеджером для целевой системы. Это запись исходного окружения, а не результат проверки на других платформах.

## 2. Сборка

Из корня репозитория:

```bash
cd ros2/src/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Собирать пакеты следует системным Python из окружения ROS 2. Виртуальное окружение YOLO задаётся отдельно при запуске детектора.

## 3. Запуск

Все команды ниже выполняются из корня репозитория в отдельных терминалах. В каждом терминале загрузите окружение:

```bash
source /opt/ros/jazzy/setup.bash
source ros2/src/ros2_ws/install/setup.bash
```

**Оборудование и одометрия:**

```bash
ros2 launch frob_bringup bringup.launch.py
```

**Камера и детектор знаков:**

```bash
ros2 launch frob_sign_detection sign_detection.launch.py \
  python_executable:="$PWD/ros2/venv-yolo/bin/python"
```

Путь к интерпретатору настраивается через `python_executable`; привязки к `/home/pi/Frob_robot` нет. Камера по умолчанию — `/dev/video0`. Разрешение, порог уверенности, размер входа модели и ограничение частоты заданы в `frob_sign_detection/launch/sign_detection.launch.py`. Исходное ограничение частоты обработки — **4 кадра/с**, что не является измерением фактической производительности.

**Узел миссии:**

```bash
ros2 launch frob_city_mission city_mission.launch.py
```

Узел запускается в состоянии `IDLE`. Проверка движения на одну ячейку и старт полной миссии вызывают движение робота:

```bash
# Проверка одной ячейки
ros2 service call /city_mission/test_cell std_srvs/srv/Trigger '{}'

# Запуск полной миссии
ros2 service call /city_mission/start std_srvs/srv/Trigger '{}'

# Запрос остановки
ros2 service call /city_mission/stop std_srvs/srv/Trigger '{}'
```

Управление остановкой передаётся через ROS 2; это не аппаратный аварийный выключатель. На время проверки обеспечьте доступ к выключателю питания робота.

## 4. Наблюдение и настройки

В браузере устройства в той же сети откройте `http://<IP-робота>:8080/` для просмотра распознавания знаков или `http://<IP-робота>:8081/` для отладки линии и состояния миссии.

| Интерфейс ROS 2 | Содержимое |
| --- | --- |
| `/camera/image_raw` | Изображение с камеры, `sensor_msgs/Image` |
| `/sign_detector/detections` | JSON с обнаруженными знаками, `std_msgs/String` |
| `/sign_detector/image/compressed` | Изображение с детекциями, `sensor_msgs/CompressedImage` |
| `/scan` | Данные лидара, `sensor_msgs/LaserScan` |
| `/city_mission/state` | Состояние миссии, `std_msgs/String` |
| `/city_mission/line_debug/compressed` | Отладочное изображение разметки |
| `/cmd_vel` | Команды скорости, `geometry_msgs/Twist` |
| `/forward_robot`, `/rotate_robot` | Сервисы дискретного перемещения и поворота |

Настройки поведения находятся в [`mission.yaml`](../ros2/src/ros2_ws/src/frob_city_mission/config/mission.yaml). В исходной конфигурации заданы скорость 0,07 м/с, остановка у знака на 2,2 с, одна ожидаемая зона посадки и дистанция остановки перед препятствием 0,25 м. Это настройки конкретного решения; их следует согласовать с роботом и полигоном.

Карта минского полигона сохранена в `frob_navigation/maps/map.pgm` и `map.yaml`. Она не является универсальной картой для других площадок.

## Проверка переноса

При подготовке обновления проверены синтаксис Python, XML-манифесты пакетов, ссылки в документации и целостность весов по контрольной сумме. В ROS 2 Jazzy собраны пакеты `frob_interfaces`, `frob_sign_detection`, `frob_city_mission` и `ros2_arduino_bridge` с системным Python `/usr/bin/python3`. Launch-файлы проверены без запуска оборудования. Прошивка Arduino, загрузка модели для инференса и движение на физическом роботе в эту проверку не входят.
