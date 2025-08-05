import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Initialize camera with better settings for USB cameras
        self.cap = None
        self.frame = None
        self.frame_lock = threading.Lock()
        
        self.init_camera()
        
        if self.cap is not None and self.cap.isOpened():
            # Start camera thread
            self.camera_thread = threading.Thread(target=self.camera_loop)
            self.camera_thread.daemon = True
            self.camera_thread.start()
            
            # Timer for publishing frames
            self.timer = self.create_timer(0.1, self.publish_frame)  # 10 FPS
            self.get_logger().info('Camera test node started - publishing to /camera/image_raw')
        else:
            self.get_logger().error('Failed to initialize camera')
    
    def init_camera(self):
        """Initialize camera with USB camera friendly settings"""
        try:
            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            
            if not self.cap.isOpened():
                self.get_logger().error('Failed to open camera at /dev/video0')
                return
            
            # Set buffer size to reduce latency
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Try different format settings
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 15)
            
            # Set exposure and other settings to auto
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Auto exposure
            
            self.get_logger().info(f'Camera initialized: {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} @ {self.cap.get(cv2.CAP_PROP_FPS)} FPS')
            
        except Exception as e:
            self.get_logger().error(f'Camera initialization error: {e}')
    
    def camera_loop(self):
        """Continuous camera capture in separate thread"""
        while rclpy.ok():
            try:
                ret, frame = self.cap.read()
                if ret:
                    with self.frame_lock:
                        self.frame = frame.copy()
                else:
                    self.get_logger().warn('Failed to read frame from camera', throttle_duration_sec=5.0)
                    time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f'Camera capture error: {e}')
                time.sleep(1.0)
    
    def publish_frame(self):
        """Publish the latest frame"""
        with self.frame_lock:
            if self.frame is not None:
                try:
                    msg = self.bridge.cv2_to_imgmsg(self.frame, 'bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera_frame'
                    self.publisher.publish(msg)
                    self.get_logger().info('Published frame', throttle_duration_sec=2.0)
                except Exception as e:
                    self.get_logger().error(f'Frame publishing error: {e}')

    def __del__(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()

def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
