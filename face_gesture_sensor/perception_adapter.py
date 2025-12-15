#!/usr/bin/env python3

from math import pi
import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Bool, String, Float32

import subprocess
import time

"""
see: https://chatgpt.com/s/t_693f44d1eca08191b70128efa562496e
     https://chatgpt.com/s/t_693f472c11488191adc7db1457b82db5

Design overview (intentional & Nav2-friendly)

  What this node does
    - Listens to perception output
    - Publishes semantic events for a Behavior Tree
    - Executes aplay for face detection
    - Publishes intent messages for motion (not raw velocity)

  What this node does NOT do
    - Does not publish /cmd_vel
    - Does not fight Nav2 controllers
    - Does not contain navigation logic
"""

class PerceptionAdapter(Node):
    """
    Perception → Behavior adapter for Face & Gesture sensor.

    Responsibilities:
      * Listen to Detection2DArray messages from face_gesture_sensor
      * Detect semantic events (FACE, LIKE, OK, STOP, YES, SIX)
      * Publish BT-friendly signals
      * Trigger sound playback for face detection
    """

    def __init__(self):
        super().__init__('perception_adapter')

        # ---- parameters ----
        self.declare_parameter('face_sound', 'my_face.wav')
        self.declare_parameter('min_confidence', 0.6)
        self.declare_parameter('face_cooldown_sec', 3.0)
        self.declare_parameter('camera_center_x', 320.0)  # Assuming 640px width sensor camera
        self.declare_parameter('face_x_threshold', 10.0)  # New: Threshold for x change to publish
        self.declare_parameter('ticker_interval_sec', 0.5)  # New: Ticker interval

        self.face_sound = self.get_parameter('face_sound').value
        self.min_conf = self.get_parameter('min_confidence').value
        self.face_cooldown = self.get_parameter('face_cooldown_sec').value
        self.camera_center_x = self.get_parameter('camera_center_x').value
        self.face_x_threshold = self.get_parameter('face_x_threshold').value
        self.ticker_interval = self.get_parameter('ticker_interval_sec').value

        # ---- state machine ----
        self.state = 'idle'  # 'idle' or 'tracking'
        self.last_face_time = 0.0
        self.last_traced_x = 0.0

        # ---- publishers (BT inputs) ----
        self.face_pub = self.create_publisher(
            Bool, '/bt/face_detected', 10
        )

        self.face_yaw_err_pub = self.create_publisher(
            Float32, '/bt/face_yaw_error', 10
        )

        self.gesture_pub = self.create_publisher(
            String, '/bt/gesture_command', 10
        )

        self.forward_pub = self.create_publisher(
            Float32, '/bt/forward_request', 10
        )

        # ---- subscription ----
        self.sub = self.create_subscription(
            Detection2DArray,
            'face_gesture_detections',  # Match the fgs_node published topic
            self._on_detection,
            10
        )

        # ---- ticker timer ----
        self.ticker_timer = self.create_timer(self.ticker_interval, self._ticker_callback)

        self.get_logger().info("Perception Adapter ready")

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------

    def _on_detection(self, msg: Detection2DArray):
        """
        Handle detections from Detection2DArray (of ObjectHypothesisWithPose messages).
        """
        for detection in msg.detections:
            for hypothesis_with_pose in detection.results:
                hypothesis = hypothesis_with_pose.hypothesis
                label = hypothesis.class_id.upper()
                confidence = hypothesis.score

                if confidence < self.min_conf:
                    continue

                if label == 'FACE':
                    # Extract face position from bbox
                    face_x = detection.bbox.center.position.x
                    self._handle_face(face_x)
                elif 'LIKE' in label: # "1 = LIKE (blue)"
                    self._handle_like()
                elif 'OK' in label:
                    self._handle_ok()
                elif 'STOP' in label:
                    self._handle_stop()
                elif 'YES' in label:
                    self._handle_yes()
                elif 'SIX' in label:
                    self._handle_six()

    def _ticker_callback(self):
        """
        Periodic check: If face not detected for face_cooldown_sec, reset to idle.
        """
        now = time.time()
        if self.state == 'tracking' and (now - self.last_face_time) > self.face_cooldown:
            self.get_logger().info("Face disappeared, resetting state to idle")
            self.state = 'idle'
            self.last_traced_x = 0.0  # Reset

    # --------------------------------------------------
    # Semantic handlers
    #
    # Each handler processes a specific gesture or face detection event.
    # They abstract away low-level details and publish high-level intent messages.
    # --------------------------------------------------

    def _handle_face(self, face_x):
        now = time.time()
        self.last_face_time = now  # Update last seen time

        distance_px = face_x - self.camera_center_x # positive turn when you are on robot's left

        self.face_pub.publish(Bool(True)) # Face detected event for BT, publish continuously while face is in view

        angle_factor = pi / (6 * 320)   # Factor to convert pixel error to angle error (assume 30 degrees = 320 pixels)
        angle_error = (face_x - self.camera_center_x) * angle_factor

        self.face_yaw_err_pub.publish(Float32(data=float(angle_error))) # Where to turn for BT

        # State Machine:
        #    'idle': No face. On detection, greet and switch to 'tracking'.
        #    'tracking': Face in view. Trace only on significant x-change.
        # State resets to 'idle' in "_ticker_callback()" if no detection for face_cooldown_sec.

        if self.state == 'idle':
            # First face detection: greet and start tracking
            self.last_traced_x = face_x # expect 0...640
            self.get_logger().info(f"Face detected (first time) at x={face_x}, distance_px: {distance_px}  angle_error: {angle_error}")

            # Play greeting sound, once per appearance
            #subprocess.Popen(['aplay', self.face_sound])
            # Or, use "flite": sudo apt install flite
            subprocess.Popen(['flite', '-t', 'Hello, I see you!'])

            self.state = 'tracking'

        elif self.state == 'tracking':
            # Check if x changed enough to publish
            if abs(face_x - self.last_traced_x) > self.face_x_threshold:
                self.last_traced_x = face_x
                self.get_logger().info(f"Face moved, x={face_x}, distance_px: {distance_px}  angle_error: {angle_error}")


    def _handle_like(self):
        self.get_logger().info("Gesture: LIKE")
        # Notify BT
        self.gesture_pub.publish(String(data='LIKE'))
        # Add custom action if needed (e.g., play sound or trigger motion)

    def _handle_ok(self):
        self.get_logger().info("Gesture: OK")
        self.gesture_pub.publish(String(data='OK'))

    def _handle_stop(self):
        self.get_logger().info("Gesture: STOP")
        self.gesture_pub.publish(String(data='STOP'))

    def _handle_yes(self):
        self.get_logger().info("Gesture: YES")
        self.gesture_pub.publish(String(data='YES'))

    def _handle_six(self):
        self.get_logger().info("Gesture: SIX")
        self.gesture_pub.publish(String(data='SIX'))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
