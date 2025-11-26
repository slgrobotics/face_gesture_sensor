Visit my [main project Wiki](https://github.com/slgrobotics/articubot_one/wiki) for more info.

## A ROS2 Node for DFRobot AI Gesture and Face Detection Sensor 

Based on [DFRobot Python code](https://github.com/DFRobot/DFRobot_GestureFaceDetection/tree/master/python/raspberrypi)

Product Link: https://www.dfrobot.com/product-2914.html

#### On a Raspberry Pi with the sensor connected to I2C:

Get code (you may want to use `~/robot_ws/src` here):
```
mkdir -p ~/gesture_ws/src
cd ~/gesture_ws/src
git clone https://github.com/slgrobotics/face_gesture_sensor.git
```

Install dependencies:
```
sudo rosdep init    # do it once, if you haven't done it before
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -r -y
```

Build it:
```
cd ~/gesture_ws
colcon build
```

Run the node:
```
ros2 run face_gesture_sensor fgs_node
```
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

