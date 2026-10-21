"""Exercise Lyrical messages and nodes without attached sensor hardware."""
from unittest.mock import Mock, patch

import pytest
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from vision_msgs.msg import Detection2DArray

from face_gesture_sensor.fgs_node import FaceGestureSensorNode
from face_gesture_sensor.perception_adapter import PerceptionAdapter


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_sensor_messages_and_adapter_confidence(ros_context):
    sensor = Mock()
    sensor.get_face_number.return_value = 1
    sensor.get_face_score.return_value = 74
    sensor.get_face_location_x.return_value = 186
    sensor.get_face_location_y.return_value = 380
    sensor.get_gesture_type.return_value = 3
    sensor.get_gesture_score.return_value = 93
    with patch('face_gesture_sensor.fgs_node.DFRobot_GestureFaceDetection_I2C',
               return_value=sensor):
        driver = FaceGestureSensorNode()
    adapter = PerceptionAdapter()
    try:
        driver.detection_pub = Mock()
        driver.setup()
        assert driver.sensor_ready
        sensor.set_face_detect_thres.assert_called_once_with(60)
        driver.loop_callback()
        msg = driver.detection_pub.publish.call_args.args[0]
        msg = deserialize_message(serialize_message(msg), Detection2DArray)
        detection = msg.detections[0]
        assert detection.header == msg.header
        assert detection.bbox.center.position.x == 186.0
        assert [r.hypothesis.score for r in detection.results] == [0.74, 0.93]
        adapter._handle_face = Mock()
        adapter._handle_stop = Mock()
        adapter._on_detection(msg)
        adapter._handle_face.assert_called_once_with(186.0)
        adapter._handle_stop.assert_called_once_with()
        adapter._handle_face.reset_mock()
        adapter._handle_stop.reset_mock()
        for result in detection.results:
            result.hypothesis.score = 0.59
        adapter._on_detection(msg)
        adapter._handle_face.assert_not_called()
        adapter._handle_stop.assert_not_called()
        sensor.get_face_number.return_value = 0
        driver.detection_pub.reset_mock()
        driver.loop_callback()
        driver.detection_pub.publish.assert_not_called()
    finally:
        adapter.destroy_node()
        driver.destroy_node()


def test_installed_launch_parameters(ros_context):
    from ament_index_python.packages import get_package_share_directory
    from pathlib import Path
    import runpy
    import yaml

    share = Path(get_package_share_directory('face_gesture_sensor'))
    launch = runpy.run_path(str(share / 'launch/face_gesture_sensor.launch.py'))
    with patch.dict(launch['generate_launch_description'].__globals__, Node=Mock()) as scope:
        launch['generate_launch_description']()
        calls = scope['Node'].call_args_list
    assert len(calls) == 2
    for call in calls:
        assert call.kwargs['parameters'] == [str(share / 'config/params.yaml')]
    params_path = share / 'config/params.yaml'
    with patch('face_gesture_sensor.fgs_node.DFRobot_GestureFaceDetection_I2C'):
        driver = FaceGestureSensorNode()
    adapter = PerceptionAdapter()
    try:
        parameters = yaml.safe_load(params_path.read_text())
        assert driver.get_name() in parameters
        assert adapter.get_name() in parameters
    finally:
        adapter.destroy_node()
        driver.destroy_node()
