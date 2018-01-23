Навигация в поле ArUco-маркеров
================================

**DEPRECATED**. Ипользуйте [Clever Bundle](https://github.com/CopterExpress/clever_bundle).

Пакет осуществляет детектирование позиции по карте ArUco-маркеров а также навигацию по ней для PX4.

Необходимо использование LPE, включенного Fusion на Vision Positoin и Vision Yaw. Подробнее см. [информацию о настройках PX4](https://github.com/CopterExpress/clever/blob/master/docs/setup.md).


Также необоходимо включить использование Vision в Q Attitude Estimator (TODO).

Публикуемые в TF системы координат:

* ``local_origin`` (MAVROS) — координаты относительно точки инициализации полетного контроллера
* ``fcu`` (MAVROS) —  координаты относительно квадрокоптера
* ``fcu_horiz`` — координаты относительно квадрокоптера без учета наклонов по танкажу и рысканью
* ``marker_map`` — координаты относительно поля ArUco-маркеров

Пример трансформации из системы координат ``local_origin`` в ``marker_map``:

```python
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point
import tf

# ...

transform_listener = tf.TransformListener()

pose = PoseStamped()
pose.header.frame_id = 'marker_map'
pose.pose.position = Point(1, 2, 3)  # координаты относительного маркерного поля

# Задаем рысканье относительного маркерного поля
q = tf.transformations.quaternion_from_euler(0, 0, math.radions(23))
pose.pose.orientation = Quaternion(*q)

# Производим трансформацию в систему координат ``local_origin``
pose_local = transform_listener.transformPose('local_origin', pose)
```

Далее можно опубликовать ``pose_local`` в топик ``/mavros/setpoint_position/local``.

```python
pose_local.header.stamp = rospy.get_rostime()
pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
pub.publish(pose_local)
```

Simple control
--------------

ROS-модуль для упрощения управления коптером через MAVROS и автоматической трансформации системы координат.
При вызове любого из сервисов коптер автоматически переведется в OFFBOARD и заармится (**коптер взлетит, если находится на полу!**)

Общие для сервисов параметры:

* ``frame_id`` — система координат в TF, в которой заданы координаты и рысканье (yaw).
* ``update_frame`` — считать ли систему координат изменяющейся (false для local_origin, fcu, true для marker_map)
* ``yaw`` — рысканье в радианах в заданой системе координат (0 – коптер смотрит по оси X).
* ``yaw_rate`` — угловая скорость по рысканью в радианах в секунду (против часовой).
* ``thrust`` — уровень газа (от 0 [нет газа] до 1 [полный газ])

Объявление прокси ко всем сервисам:

```python
import rospy
from marker_navigator.srv import SetPosition, \
    SetPositionYawRate, \
    SetVelocity, \
    SetVelocityYawRate, \
    SetAttitude, \
    SetAttitudeYawRate, \
    SetRatesYaw, \
    SetRates
from std_srvs.srv import Trigger


rospy.init_node('foo')

# Создаем прокси ко всем сервисам:

set_position = rospy.ServiceProxy('/set_position', SetPosition)
set_position_yaw_rate = rospy.ServiceProxy('/set_position/yaw_rate', SetPositionYawRate)

set_velocity = rospy.ServiceProxy('/set_velocity', SetVelocity)
set_velocity_yaw_rate = rospy.ServiceProxy('/set_Velocity/yaw_rate', SetVelocityYawRate)

set_attitude = rospy.ServiceProxy('/set_attitude', SetAttitude)
set_attitude_yaw_rate = rospy.ServiceProxy('/set_attitude/yaw_rate', SetattitudeYawRate)

set_rates_yaw = rospy.ServiceProxy('/set_rates/yaw', SetRatesYaw)
set_rates = rospy.ServiceProxy('/set_rates', SetRates)

release = rospy.ServiceProxy('/release', Trigger)
```

### set_position

Установить позицию и рысканье.

Параметры: x, y, z, yaw, frame_id, update_frame

Задание позиции относительно коптера:

```python
set_position(x=0, y=0, z=3, frame_id='fcu_horiz')  #  взлет на 3 метра
```

```python
set_position(x=1, y=0, z=0, frame_id='fcu_horiz')  # пролететь вперед на 1 метр
```

```python
set_position(x=0, y=-1, z=0, frame_id='fcu_horiz')  # пролететь вправо на 1 метр
```

Задание позиции относительно системы маркеров
(фрейм marker_map не будет опубликован, пока коптер хоть раз не увидит один из маркеров):

```python
set_position(x=2, y=2, z=3, frame_id='marker_map', update_frame=True)  #  полет в координату 2:2, высота 3 метра
```

### set_position_yaw_rate

Установить позицию и угловую скорость по рысканью.

Параметры: x, y, z, yaw_rate, frame_id, update_frame

### set_velocity

Установить скорости и рысканье.

Параметры: vx, vy, vz, yaw, frame_id, update_frame

Полет по кругу:

```python
set_velocity_yaw_rate(vx=0.2, vy=0.0, vz=0, yaw_rate=0.5, frame_id: 'fcu_horiz', update_frame: true)
```

### set_velocity_yaw_rate

Установить скорости и угловую скорость по рысканью.

Параметры: vx, vy, vz, yaw_rate, frame_id, update_frame

### set_attitude

Установить тангаж, крен, рысканье и уровень газа.

Параметры: pitch, roll, yaw, thrust, frame_id, update_frame

### set_attitude_yaw_rate

Установить тангаж, крен, угловую скорость по рысканью и уровень газа.  **Возможно, не поддерживается в PX4**.

Параметры: pitch, roll, yaw_rate, thrust

### set_rates_yaw

Установить угловые скорости по тангажу и крену, рысканье и уровень газа.

Параметры: pitch_rate, roll_rate, yaw, thrust, frame_id, update_frame

### set_rates

Установить угловые скорости по тагажу, крену и рысканью и уровень газа.

Параметры: pitch_rate, roll_rate, yaw_rate, thrust

### release

Перестать публиковать команды коптеру (отпустить управление).
Возможно продолжение управления средствами MAVROS.

Посадка
-------

Для посадки можно использовать режим ``AUTO.LAND``. Land detector должен быть включен и указан в ``LPE_FUSION``.

```python
from mavros_msgs.srv import SetMode

# ...

set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)  # объявляем прокси к сервису переключения режимов

# ...

set_mode(base_mode=0, custom_mode='AUTO.LAND')  # включаем режим посадки
```

Дебаг
---

Вывод координат коптера относительно маркерного поля:

```bash
rostopic echo /vision_position_estimator/position/marker_map
```

Статус запущенного модуля:

```bash
sudo systemctl status marker_navigator
```
