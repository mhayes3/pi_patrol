import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import subprocess
import threading
import time
import os
from datetime import datetime

class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')
        
        # ROS2 setup
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.alert_subscription = self.create_subscription(
            String, '/intruder_alert', self.alert_callback, 10)
        
        # Recording settings
        self.recording_duration = 7  # seconds
        self.recordings_dir = os.path.expanduser("~/tracking_recordings")
        self.temp_dir = "/tmp/tracking_temp"
        
        # Create directories
        os.makedirs(self.recordings_dir, exist_ok=True)
        os.makedirs(self.temp_dir, exist_ok=True)
        
        # Recording state
        self.is_recording = False
        self.current_writer = None
        self.recording_start_time = None
        self.frame_buffer = []
        self.buffer_lock = threading.Lock()
        self.max_buffer_size = 150  # ~5 seconds at 30fps
        
        # Video settings
        self.fps = 10
        self.frame_width = 640
        self.frame_height = 480
        
        self.get_logger().info(f'Recorder node initialized. Recordings saved to: {self.recordings_dir}')
    
    def image_callback(self, msg):
        """Store frames in buffer for potential recording"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Add frame to buffer (keep recent frames for pre-recording)
            with self.buffer_lock:
                if len(self.frame_buffer) >= self.max_buffer_size:
                    self.frame_buffer.pop(0)  # Remove oldest frame
                self.frame_buffer.append((cv_image.copy(), time.time()))
            
            # If currently recording, write frame
            if self.is_recording and self.current_writer is not None:
                self.current_writer.write(cv_image)
                
                # Check if recording duration reached
                if time.time() - self.recording_start_time >= self.recording_duration:
                    self.stop_recording()
        
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
    
    def alert_callback(self, msg):
        """Start recording when intruder alert received"""
        if "INTRUDER_DETECTED" in msg.data and not self.is_recording:
            self.get_logger().info(f'Intruder alert received: {msg.data}')
            self.start_recording()
    
    def start_recording(self):
        """Start video recording with buffered frames"""
        if self.is_recording:
            self.get_logger().warn('Already recording, ignoring new request')
            return
        
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            temp_filename = os.path.join(self.temp_dir, f"recording_{timestamp}.avi")
            final_filename = os.path.join(self.recordings_dir, f"intruder_{timestamp}.mp4")
            
            # Initialize video writer
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.current_writer = cv2.VideoWriter(
                temp_filename, fourcc, self.fps, 
                (self.frame_width, self.frame_height)
            )
            
            if not self.current_writer.isOpened():
                self.get_logger().error('Failed to open video writer')
                return
            
            self.is_recording = True
            self.recording_start_time = time.time()
            self.temp_filename = temp_filename
            self.final_filename = final_filename
            
            # Write buffered frames (pre-recording)
            with self.buffer_lock:
                self.get_logger().info(f'Writing {len(self.frame_buffer)} buffered frames')
                for frame, timestamp in self.frame_buffer:
                    # Resize frame if needed
                    frame_resized = cv2.resize(frame, (self.frame_width, self.frame_height))
                    self.current_writer.write(frame_resized)
            
            self.get_logger().info(f'Started recording: {final_filename}')
        
        except Exception as e:
            self.get_logger().error(f'Failed to start recording: {e}')
            self.is_recording = False
    
    def stop_recording(self):
        """Stop recording and convert to MP4"""
        if not self.is_recording:
            return
        
        try:
            self.is_recording = False
            
            # Release video writer
            if self.current_writer is not None:
                self.current_writer.release()
                self.current_writer = None
            
            # Convert AVI to MP4 using FFmpeg
            self.get_logger().info('Converting recording to MP4...')
            conversion_thread = threading.Thread(
                target=self.convert_to_mp4, 
                args=(self.temp_filename, self.final_filename)
            )
            conversion_thread.daemon = True
            conversion_thread.start()
            
            duration = time.time() - self.recording_start_time
            self.get_logger().info(f'Recording stopped. Duration: {duration:.1f} seconds')
        
        except Exception as e:
            self.get_logger().error(f'Error stopping recording: {e}')
    
    def convert_to_mp4(self, temp_file, final_file):
        """Convert AVI to MP4 using FFmpeg"""
        try:
            # Strictly enforce 7 second duration in the output video
            cmd = [
                'ffmpeg', '-y',  # -y to overwrite existing files
                '-i', temp_file,
                '-c:v', 'libx264',
                '-preset', 'fast',
                '-crf', '23',
                '-pix_fmt', 'yuv420p',  # Required for compatibility
                '-movflags', '+faststart',  # Optimize for web streaming
                '-profile:v', 'baseline',  # Most compatible profile
                '-level', '3.0',  # Compatibility level
                '-an',  # No audio (since we don't have audio)
                '-t', '7',  # Strictly limit to 7 seconds
                final_file
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.get_logger().info(f'Recording saved: {final_file}')
                # Remove temporary file
                os.remove(temp_file)
            else:
                self.get_logger().error(f'FFmpeg conversion failed: {result.stderr}')
        
        except Exception as e:
            self.get_logger().error(f'Video conversion error: {e}')

def main():
    rclpy.init()
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.is_recording:
            node.stop_recording()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
