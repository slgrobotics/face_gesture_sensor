import rclpy
from rclpy.node import Node

#
# ros2 run face_gesture_sensor fgs_node
#

class FaceGestureSensorNode(Node):
    def __init__(self):
        super().__init__('face_gesture_sensor_node')  # Initialize the Node with a unique name
        self.get_logger().info('FaceGestureSensorNode node has been started!')
        
        # Add your node's logic here (publishers, subscribers, timers, etc.)

def main(args=None):
    rclpy.init(args=args)
    node = FaceGestureSensorNode()
    rclpy.spin(node)  # Keep the node alive and processing callbacks
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

