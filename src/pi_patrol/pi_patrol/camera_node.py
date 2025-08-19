import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import threading
import time
import os

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Initialize camera with better settings for USB cameras
        self.cap = None
        self.frame = None
        self.frame_lock = threading.Lock()
        self.cap_lock = threading.Lock()
        self.paused_by_stream = False  # retained for compatibility; no longer used to pause capture
        
        # FIFO streaming support (for concurrent detection + streaming)
        self.fifo_path = '/tmp/pi_patrol_frames'
        self.fifo_writer_thread = None
        self.fifo_stop_event = threading.Event()
        self.fifo_fps = 15
        
        self.init_camera()
        
        # Start camera thread regardless; it will wait if camera is not available
        self.camera_thread = threading.Thread(target=self.camera_loop)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        
        # Timer for publishing frames
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 FPS
        if self.cap is not None and self.cap.isOpened():
            self.get_logger().info('Camera test node started - publishing to /camera/image_raw')
        else:
            self.get_logger().warn('Camera not available at startup; will retry when instructed')

        # Listen for server commands to control FIFO writer when streaming starts/stops
        self.command_sub = self.create_subscription(String, '/server/command', self.on_command, 10)
    
    def init_camera(self):
        """Initialize camera with USB camera friendly settings"""
        try:
            # Try multiple camera indices and methods
            # First try with index 0
            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            
            # If that fails, try with index 1
            if not self.cap.isOpened():
                self.get_logger().info('Failed with index 0, trying index 1')
                self.cap.release()
                self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
            
            # If that fails too, try with direct device path
            if not self.cap.isOpened():
                self.get_logger().info('Failed with index 1, trying direct device path')
                self.cap.release()
                self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
            
            # If still not working, try without specifying backend
            if not self.cap.isOpened():
                self.get_logger().info('Failed with direct path, trying without V4L2 backend')
                self.cap.release()
                self.cap = cv2.VideoCapture(0)
            
            if not self.cap.isOpened():
                self.get_logger().error('Failed to open camera with all methods')
                return
            
            # Set buffer size to reduce latency
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Try different format settings: prefer raw YUYV to avoid JPEG decoder warnings
            yuyv_fourcc = cv2.VideoWriter_fourcc('Y','U','Y','V')
            mjpg_fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
            yuyv_ok = False
            try:
                yuyv_ok = self.cap.set(cv2.CAP_PROP_FOURCC, yuyv_fourcc)
            except Exception:
                yuyv_ok = False
            if not yuyv_ok:
                self.get_logger().warn('YUYV not supported by camera, falling back to MJPG')
                self.cap.set(cv2.CAP_PROP_FOURCC, mjpg_fourcc)
            
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
                with self.cap_lock:
                    cap = self.cap
                if cap is None or not cap.isOpened():
                    time.sleep(0.2)
                    continue
                ret, frame = cap.read()
                if ret:
                    with self.frame_lock:
                        self.frame = frame.copy()
                else:
                    self.get_logger().warn('Failed to read frame from camera', throttle_duration_sec=5.0)
                    time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f'Camera capture error: {e}')
                time.sleep(1.0)

    def stop_capture(self):
        """Release the camera device so other processes can use it."""
        try:
            with self.cap_lock:
                if self.cap is not None:
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    self.cap = None
            self.get_logger().info('Camera capture released')
        except Exception as e:
            self.get_logger().error(f'Error releasing camera: {e}')

    def start_capture(self):
        """Reopen the camera device if not currently opened."""
        with self.cap_lock:
            need_open = (self.cap is None) or (not self.cap.isOpened())
        if need_open:
            self.init_camera()
            if self.cap is not None and self.cap.isOpened():
                self.get_logger().info('Camera capture re-initialized')
            else:
                self.get_logger().warn('Failed to re-initialize camera')

    def on_command(self, msg: String):
        cmd = (msg.data or '').strip()
        if cmd == 'start_stream':
            # Start FIFO writer to feed ffmpeg while keeping capture running
            self.start_fifo_writer()
        elif cmd == 'stop_stream':
            # Stop FIFO writer, keep capture for detection
            self.stop_fifo_writer()
    
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

    def start_fifo_writer(self):
        """Start a background thread that writes raw BGR frames to a named FIFO for ffmpeg."""
        try:
            # Ensure FIFO exists
            if not os.path.exists(self.fifo_path):
                try:
                    os.mkfifo(self.fifo_path)
                    self.get_logger().info(f'Created FIFO at {self.fifo_path}')
                except FileExistsError:
                    pass
                except Exception as e:
                    self.get_logger().error(f'Failed to create FIFO {self.fifo_path}: {e}')
                    return
            # If already running, do nothing
            if self.fifo_writer_thread and self.fifo_writer_thread.is_alive():
                return
            self.fifo_stop_event.clear()

            def _writer_loop():
                self.get_logger().info(f'Starting FIFO writer to {self.fifo_path}')
                try:
                    # Open FIFO for blocking write until ffmpeg connects
                    with open(self.fifo_path, 'wb', buffering=0) as f:
                        last_time = 0.0
                        period = max(1.0 / float(self.fifo_fps), 0.01)
                        while not self.fifo_stop_event.is_set():
                            frame_bytes = None
                            with self.frame_lock:
                                if self.frame is not None:
                                    try:
                                        frame_bytes = self.frame.tobytes()
                                        # Optionally check shape
                                    except Exception:
                                        frame_bytes = None
                            if frame_bytes is not None:
                                try:
                                    f.write(frame_bytes)
                                except BrokenPipeError:
                                    self.get_logger().warn('FIFO writer: broken pipe; reader closed')
                                    break
                                except Exception as e:
                                    self.get_logger().error(f'FIFO writer error: {e}')
                                    break
                            # pace writes
                            now = time.time()
                            if now - last_time < period:
                                time.sleep(period - (now - last_time))
                            last_time = now
                except FileNotFoundError:
                    self.get_logger().warn(f'FIFO {self.fifo_path} not found when starting writer')
                except Exception as e:
                    self.get_logger().error(f'FIFO writer setup error: {e}')
                finally:
                    self.get_logger().info('FIFO writer stopped')

            self.fifo_writer_thread = threading.Thread(target=_writer_loop, daemon=True)
            self.fifo_writer_thread.start()
        except Exception as e:
            self.get_logger().error(f'Error starting FIFO writer: {e}')

    def stop_fifo_writer(self):
        """Signal FIFO writer to stop and wait briefly."""
        try:
            self.fifo_stop_event.set()
            th = self.fifo_writer_thread
            if th and th.is_alive():
                th.join(timeout=1.0)
        except Exception as e:
            self.get_logger().error(f'Error stopping FIFO writer: {e}')

    def __del__(self):
        if hasattr(self, 'cap') and self.cap is not None:
            with getattr(self, 'cap_lock', threading.Lock()):
                try:
                    self.cap.release()
                except Exception:
                    pass

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
