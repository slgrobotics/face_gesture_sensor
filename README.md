## A ROS2 Node for DFRobot AI Gesture and Face Detection Sensor 

Based on [DFRobot Python code](https://github.com/DFRobot/DFRobot_GestureFaceDetection/tree/master/python/raspberrypi)

Product Link: https://www.dfrobot.com/product-2914.html

Get code (you may want to use `~/robot_ws/src` here):
```
mkdir -p ~/gesture_ws/src
cd ~/gesture_ws/src
git clone https://github.com/slgrobotics/face_gesture_sensor.git
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

