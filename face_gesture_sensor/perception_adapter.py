#!/usr/bin/env python3

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
        self.declare_parameter('camera_center_x', 0.0)
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
        self.last_published_x = 0.0

        # ---- publishers (BT inputs) ----
        self.face_pub = self.create_publisher(
            String, '/bt/face_detected', 10  # Change to String
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
                elif label == 'LIKE':
                    self._handle_like()
                elif label == 'OK':
                    self._handle_ok()
                elif label == 'STOP':
                    self._handle_stop()
                elif label == 'YES':
                    self._handle_yes()
                elif label == 'SIX':
                    self._handle_six()

    def _ticker_callback(self):
        """
        Periodic check: If face not detected for face_cooldown_sec, reset to idle.
        """
        now = time.time()
        if self.state == 'tracking' and (now - self.last_face_time) > self.face_cooldown:
            self.get_logger().info("Face disappeared, resetting state to idle")
            self.state = 'idle'
            self.last_published_x = 0.0  # Reset

    # --------------------------------------------------
    # Semantic handlers
    #
    # Each handler processes a specific gesture or face detection event.
    # They abstract away low-level details and publish high-level intent messages.
    # --------------------------------------------------

    def _handle_face(self, face_x):
        now = time.time()
        self.last_face_time = now  # Update last seen time

        # State Machine:
        #    'idle': No face. On detection, greet, publish, and switch to 'tracking'.
        #    'tracking': Face in view. Publish only on significant x-change.
        # State resets to 'idle' in "_ticker_callback()" if no detection for face_cooldown_sec.

        if self.state == 'idle':
            # First face detection: greet and start tracking
            self.last_published_x = face_x
            distance = abs(face_x - self.camera_center_x)
            self.get_logger().info(f"Face detected (first time) at x={face_x}, distance: {distance}")
            self.face_pub.publish(String(data=f"FACE {distance}"))

            # Play greeting sound, once per appearance
            #subprocess.Popen(['aplay', self.face_sound])
            # Or, use "flite": sudo apt install flite
            subprocess.Popen(['flite', '-t', 'Hello, I see you!'])

            self.state = 'tracking'

        elif self.state == 'tracking':
            # Check if x changed enough to publish
            if abs(face_x - self.last_published_x) > self.face_x_threshold:
                self.last_published_x = face_x
                distance = abs(face_x - self.camera_center_x)
                self.get_logger().info(f"Face moved, publishing at x={face_x}, distance: {distance}")
                self.face_pub.publish(String(data=f"FACE {distance}"))


    def _handle_like(self):
        self.get_logger().info("Gesture: LIKE")
        # Notify BT
        self.gesture_pub.publish(String(data='LIKE'))
        # Add custom action if needed (e.g., play sound or trigger motion)

    def _handle_ok(self):
        self.get_logger().info("Gesture: OK")

        # Notify BT
        self.gesture_pub.publish(String(data='OK'))

        # Request forward motion for 1 second
        self.forward_pub.publish(Float32(data=1.0))

    def _handle_stop(self):
        self.get_logger().info("Gesture: STOP")

        # Notify BT
        self.gesture_pub.publish(String(data='STOP'))

    def _handle_yes(self):
        self.get_logger().info("Gesture: YES")
        # Notify BT
        self.gesture_pub.publish(String(data='YES'))
        # Add custom action if needed

    def _handle_six(self):
        self.get_logger().info("Gesture: SIX")
        # Notify BT
        self.gesture_pub.publish(String(data='SIX'))
        # Add custom action if needed


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
