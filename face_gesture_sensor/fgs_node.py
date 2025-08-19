import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from datetime import datetime

from .utils.DFRobot_GestureFaceDetection import DFRobot_GestureFaceDetection_I2C, DFRobot_GestureFaceDetection_UART

from std_msgs.msg import Header
from vision_msgs.msg import Point2D, Pose2D, BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose, ObjectHypothesis

#
# ros2 run face_gesture_sensor fgs_node
#

# Macro definition: Set to True to use I2C, False to use UART
USE_I2C = True  # Set to True to use I2C, False to use UART

# Define device address and baud rate
DEVICE_ID = 0x72
UART_BAUD_RATE = 9600

# Choose between I2C or UART based on the macro definition
if USE_I2C:
    # Using I2C interface
    gfd = DFRobot_GestureFaceDetection_I2C(bus=1, addr=DEVICE_ID)  # Assuming I2C bus 1 is used
else:
    # Using UART interface
    gfd = DFRobot_GestureFaceDetection_UART(baud=UART_BAUD_RATE, addr=DEVICE_ID)


class FaceGestureSensorNode(Node):
    def __init__(self):
        super().__init__('face_gesture_sensor_node')  # Initialize the Node with a unique name
        self.sensor_ready = False  # Flag to indicate sensor is initialized
        self.get_logger().info('FaceGestureSensorNode node has been started!')
        self.create_timer(5.0, self.setup)  # Call setup after 5 seconds
        self.create_timer(0.5, self.loop_callback)  # Call every 500 ms
        self.detection_pub = self.create_publisher(Detection2DArray, 'face_gesture_detections', 10)  # Add publisher
        self.print_time_counter = 0  # Add a counter for print_time()
        self.start_time = datetime.now()  # Store program start time
        
    def setup(self):
        """
        @brief Setup function for initializing sensor thresholds.
        
        This function gets the thresholds for face detection and gesture detection.
        """
        # This will be called after 5 seconds - let the sensor to start.

        while gfd.begin() == False:
            self.get_logger().error("Communication with device failed, please check connection")
            rclpy.shutdown()
            return
        
        self.setup_timer.cancel()  # Cancel the timer after first call

        '''
        # Set face detection score threshold (0~100)
        if gfd.set_face_detect_thres(60):
            self.get_logger().info("Face detection threshold set to 60.")
        else:
            self.get_logger().error("Set the face detection threshold fail.")
            rclpy.shutdown()
            return

        # Set gesture detection score threshold (0~100)
        if gfd.set_gesture_detect_thres(60):
            self.get_logger().info("Gesture detection threshold set to 60.")
        else:
            self.get_logger().error("Set the gesture detection threshold fail.")
            rclpy.shutdown()
            return

        # Set detection range, 0~100
        if gfd.set_detect_thres(100):
            self.get_logger().info("Detection range set to maximum.")
        else:
            self.get_logger().error("Set the gesture detection range fail.")
            rclpy.shutdown()
            return
        '''

        # Get face detection score threshold (0~100)
        self.get_logger().info("face detection threshold: {}".format(gfd.get_face_detect_thres()))

        # Get gesture detection score threshold (0~100)
        self.get_logger().info("gesture detection threshold: {}".format(gfd.get_gesture_detect_thres()))

        # Get detection range, 0~100
        self.get_logger().info("gesture detection range: {}".format(gfd.get_detect_thres()))

        self.sensor_ready = True

    def loop_callback(self):
        # This method will be called every 500 ms
        #self.get_logger().info('Periodic callback triggered')

        if not self.sensor_ready:
            return

        self.print_time()

        # Check if any faces are detected
        if gfd.get_face_number() > 0:
            # Get face score and position coordinates
            face_score = gfd.get_face_score()
            face_x = gfd.get_face_location_x()
            face_y = gfd.get_face_location_y()
            
            self.get_logger().info("Detect face at (x = {}, y = {}, score = {})".format(face_x, face_y, face_score))
            
            # Get gesture type and score
            # - 1: LIKE  - blue LED color
            # - 2: OK    - green
            # - 3: STOP  - red
            # - 4: YES   - yellow
            # - 5: SIX   - purple
            gesture_type = gfd.get_gesture_type()
            gesture_score = gfd.get_gesture_score()
            
            self.get_logger().info("Detect gesture {}, score = {}".format(gesture_type, gesture_score))

            detection_array_msg = Detection2DArray()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "face_gesture_sensor"  # or another appropriate frame
            detection_array_msg.header = header

            center = Pose2D()
            center.position = Point2D(x=float(face_x), y=float(face_y))
            center.theta = 0.0  # Assuming theta is not used for 2D detection
            face_bbox = BoundingBox2D(center=center, size_x=10.0, size_y=10.0)
            detection = Detection2D(bbox=face_bbox)

            # Fill face_score in results
            hypothesis_face = ObjectHypothesisWithPose()
            hypothesis_face.hypothesis = ObjectHypothesis()
            hypothesis_face.hypothesis.score = float(face_score)
            hypothesis_face.hypothesis.class_id = "face"
            detection.results.append(hypothesis_face)

            # Fill gesture_score in results
            hypothesis_gesture = ObjectHypothesisWithPose()
            hypothesis_gesture.hypothesis = ObjectHypothesis()
            hypothesis_gesture.hypothesis.score = float(gesture_score)
            hypothesis_gesture.hypothesis.class_id = str(gesture_type)
            detection.results.append(hypothesis_gesture)

            detection_array_msg.detections.append(detection)
            self.detection_pub.publish(detection_array_msg)  # Publish the message

    def print_time(self):
        self.print_time_counter += 1
        if self.print_time_counter % 10 == 0:
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            elapsed = now - self.start_time
            elapsed_str = str(elapsed).split('.')[0]  # Format as H:M:S
            self.get_logger().info(f"Current Time: {current_time} | Elapsed: {elapsed_str}")
            
    def destroy_node(self):
        self.get_logger().info("Destroying node...")
        #gfd.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FaceGestureSensorNode()
    try:
        rclpy.spin(node)  # Keep the node alive and processing callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

