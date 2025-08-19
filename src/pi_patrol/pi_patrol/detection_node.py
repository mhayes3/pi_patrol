#!/usr/bin/env python3
import sys
import os

# Add virtual environment to Python path
venv_path = os.path.expanduser("~/ros_ml_env/lib/python3.12/site-packages")
if os.path.exists(venv_path) and venv_path not in sys.path:
    sys.path.insert(0, venv_path)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import time

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        
        # Set YOLO config directory to avoid permission issues
        os.environ['YOLO_CONFIG_DIR'] = '/tmp'
        
        # Initialize YOLO model (nano version for better Pi performance)
        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO('yolov8n.pt')  # Downloads automatically on first run
        
        #self.model.to('cpu') # Force CPU usage to avoid CUDA conflicts
        self.get_logger().info('YOLO model loaded successfully on CPU')
        
        # ROS2 setup
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        # Listen for arm/disarm commands via server command channel
        self.command_subscription = self.create_subscription(
            String, '/server/command', self.on_command, 10)
        self.detection_publisher = self.create_publisher(
            String, '/intruder_alert', 10)
        self.bbox_publisher = self.create_publisher(
            RegionOfInterest, 'detection/target_bbox', 10)
        
        # Detection settings
        self.target_classes = {0: 'person', 15: 'cat', 16: 'dog'}  # COCO class IDs
        self.confidence_threshold = 0.5
        self.alert_cooldown_sec = 10.0
        self.last_person_alert_ts = 0.0
        
        # Armed state (default disarmed). Can be set at launch via param 'armed_on_start'
        self.declare_parameter('armed_on_start', False)
        self.armed = bool(self.get_parameter('armed_on_start').value)
        
        self.get_logger().info('Detection node initialized')
    
    def image_callback(self, msg):
        try:
            # Skip processing if not armed to save CPU
            if not self.armed:
                return
            
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run detection
            results = self.model(cv_image, verbose=False)
            
            # Process detections
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])
                        
                        if class_id in self.target_classes and confidence > self.confidence_threshold:
                            class_name = self.target_classes[class_id]
                            
                            # Get bounding box coordinates
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            
                            self.get_logger().info(
                                f'Detected {class_name} with confidence {confidence:.2f}')
                            
                            # Publish alert for person detection
                            if class_name == 'person':
                                now = time.time()
                                if now - self.last_person_alert_ts >= self.alert_cooldown_sec:
                                    self.last_person_alert_ts = now
                                    alert_msg = String()
                                    alert_msg.data = f'INTRUDER_DETECTED: {class_name} at ({int(x1)},{int(y1)})'
                                    self.detection_publisher.publish(alert_msg)
                                else:
                                    self.get_logger().debug('Person detected but alert suppressed due to cooldown')

                                # Also follow the person
                                bbox_msg = RegionOfInterest()
                                bbox_msg.x_offset = int(x1)
                                bbox_msg.y_offset = int(y1)
                                bbox_msg.width = int(x2 - x1)
                                bbox_msg.height = int(y2 - y1)
                                self.bbox_publisher.publish(bbox_msg)
                            elif class_name in ['cat', 'dog']:
                                self.get_logger().info(f'Animal detected: {class_name} - publishing bounding box for tracking.')
                                bbox_msg = RegionOfInterest()
                                bbox_msg.x_offset = int(x1)
                                bbox_msg.y_offset = int(y1)
                                bbox_msg.width = int(x2 - x1)
                                bbox_msg.height = int(y2 - y1)
                                self.bbox_publisher.publish(bbox_msg)
        
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')

    def on_command(self, msg: String):
        try:
            cmd = (msg.data or '').strip().lower()
            if cmd == 'arm':
                self.armed = True
                self.get_logger().info('Detection armed')
            elif cmd == 'disarm':
                self.armed = False
                self.get_logger().info('Detection disarmed')
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    detection_node = DetectionNode()
    try:
        rclpy.spin(detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
