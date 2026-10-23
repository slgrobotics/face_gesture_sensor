Visit my [main project Wiki](https://github.com/slgrobotics/articubot_one/wiki) for more info.

## A ROS 2 Lyrical Driver for DFRobot AI Gesture and Face Detection Sensor

Based on [DFRobot Python code](https://github.com/DFRobot/DFRobot_GestureFaceDetection/tree/master/python/raspberrypi)

Works with Nav2 / Behavior Trees - custom plugins [here](https://github.com/slgrobotics/slg_bt_plugins).

Product Link: https://www.dfrobot.com/product-2914.html

Targets ROS 2 Lyrical on Ubuntu 26.04 with Python 3.14. Install ROS 2
Lyrical first, then enable I2C on the Raspberry Pi and give your user access
to `/dev/i2c-1` (typically through the `i2c` group). The default transport is
I2C bus 1, address `0x72`; UART remains selectable through `USE_I2C` in
`fgs_node.py` and uses `/dev/ttyAMA0` at 9600 baud.

#### On a Raspberry Pi with the sensor connected to I2C:

Get code (you may want to use `~/robot_ws/src` here):
```
mkdir -p ~/gesture_ws/src
cd ~/gesture_ws/src
git clone https://github.com/slgrobotics/face_gesture_sensor.git
```

Install dependencies:
```
source /opt/ros/lyrical/setup.bash
cd ~/gesture_ws
sudo rosdep init    # do it once, if you haven't done it before
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro lyrical -y
```

Build it:
```
cd ~/gesture_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select face_gesture_sensor
```

Run the node:
```
source /opt/ros/lyrical/setup.bash
source ~/gesture_ws/install/setup.bash

ros2 run face_gesture_sensor fgs_node
  or
ros2 launch face_gesture_sensor face_gesture_sensor.launch.py

```
The launch file starts both the driver and perception adapter and loads
`config/params.yaml` for both. Running `fgs_node` alone uses its defaults;
pass `--ros-args --params-file <path-to-params.yaml>` to override them.
The sensor initializes after a five-second startup delay.

Detections are published on `face_gesture_detections` as
`vision_msgs/Detection2DArray`, with pixel coordinates and confidence scores
in **0.0–1.0**. Sensor threshold parameters still use **1–100** percentages.
Consumers of earlier versions that expected 0–100 message scores must adjust
their thresholds. Existing gesture labels and BT topics are preserved.
Audio uses `aplay` and `flite`, installed by rosdep.

Run the hardware-independent compatibility tests after building:

```bash
source ~/gesture_ws/install/setup.bash
colcon test --packages-select face_gesture_sensor --python-testing pytest
colcon test-result --verbose
```

Runtime logs report face appearance/loss, gesture changes, and position changes
of at least `log_position_change_px` pixels (default `20.0`, must be positive).
Position changes are measured from the last logged position, so slow movement
still gets reported. Confidence jitter does not trigger logs. Detection and BT
messages continue at their existing rate. The default applies to both nodes,
including when launched directly by Seggy; override it with ROS parameters.

### Nav2 Behavior Trees integration

Please refer to [main project Wiki](https://github.com/slgrobotics/articubot_one/wiki/Behavior-Tree-for-Gesture-and-Face-Detection-Sensor)

------------------

### Useful links and notes

Product:
- https://www.dfrobot.com/product-2914.html
- https://www.amazon.com/dp/B0F6C91FCY

Wiki: 
- https://wiki.dfrobot.com/SKU_SEN0626_Gesture_and_Face_Detection_Module

Arduino library:
- https://github.com/DFRobot/DFRobot_GestureFaceDetection  <- installs with Arduino IDE Manager, with RTU
- https://github.com/DFRobot/DFRobot_RTU

------------------

#### Playing with [Python samples](https://github.com/DFRobot/DFRobot_GestureFaceDetection/tree/master/python/raspberrypi)

<img width="337" height="440" alt="Screenshot from 2025-11-11 22-10-15" src="https://github.com/user-attachments/assets/e65cba96-ebc3-4dd5-a933-f6cb13ba6c4d" />

```
mkdir ~/gesture
cd ~/gesture
git clone https://github.com/DFRobot/DFRobot_GestureFaceDetection.git
cd DFRobot_GestureFaceDetection
cd python/raspberrypi/examples
sudo apt install python3-smbus2

python3 get_pid_vid.py 
       FileNotFoundError: [Errno 2] No such file or directory: '/dev/ttyAMA0'
  (edited get_pid_vid.py - set USE_I2C = True)
python3 get_pid_vid.py  
When I moved in camera view, it printed and exited:
  PID: 626
  VID: 13123
  
After similarly editing,  
python3 detect_gesture.py 
face detection threshold: 60
gesture detection threshold: 60
gesture detection range: 100
Detect face at (x = 186, y = 380, score = 74)
Detect gesture 0, score = 0
...
Detect gesture 3, score = 93
Detect face at (x = 179, y = 383, score = 68)
...
Detect gesture 1, score = 85
Detect face at (x = 212, y = 365, score = 83)
```
----------

Visit my [Robots Notes repository](https://github.com/slgrobotics/robots_bringup)

