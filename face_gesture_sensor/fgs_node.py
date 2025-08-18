import rclpy
from rclpy.node import Node
from rclpy.timer import Timer

from .utils.DFRobot_GestureFaceDetection import DFRobot_GestureFaceDetection_I2C, DFRobot_GestureFaceDetection_UART

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
        
    def setup():
        """
        @brief Setup function for initializing sensor thresholds.
        
        This function gets the thresholds for face detection and gesture detection.
        """
        # This will be called after 5 seconds - let the sensor to start.

        while gfd.begin() == False:
            self.get_logger().error("Communication with device failed, please check connection")
            rclpy.shutdown()
            return

        self.sensor_ready = True

        # Set face detection score threshold (0~100)
        self.get_logger().info("face detection threshold: {}".format(gfd.get_face_detect_thres()))

        # Set gesture detection score threshold (0~100)
        self.get_logger().info("gesture detection threshold: {}".format(gfd.get_gesture_detect_thres()))

        # Set detection range, 0~100
        self.get_logger().info("gesture detection range: {}".format(gfd.get_detect_thres()))

    def loop_callback(self):
        # This method will be called every 500 ms
        self.get_logger().info('Periodic callback triggered')

        if not self.sensor_ready:
            return

        # Check if any faces are detected
        if gfd.get_face_number() > 0:
            # Get face score and position coordinates
            face_score = gfd.get_face_score()
            face_x = gfd.get_face_location_x()
            face_y = gfd.get_face_location_y()
            
            self.get_logger().info("Detect face at (x = {}, y = {}, score = {})".format(face_x, face_y, face_score))
            
            # Get gesture type and score
            # - 1: LIKE  - blue
            # - 2: OK  - green
            # - 3: STOP  - red
            # - 4: YES - yellow
            # - 5: SIX  - purple
            gesture_type = gfd.get_gesture_type()
            gesture_score = gfd.get_gesture_score()
            
            self.get_logger().info("Detect gesture {}, score = {}".format(gesture_type, gesture_score))
        


def main(args=None):
    rclpy.init(args=args)
    node = FaceGestureSensorNode()
    rclpy.spin(node)  # Keep the node alive and processing callbacks
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

