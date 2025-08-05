import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
import os
from datetime import datetime
from dotenv import load_dotenv
from pathlib import Path

class TelegramNotifier(Node):
    def __init__(self):
        super().__init__('telegram_notifier')
        
        # Load environment variables from .env file
        # Try multiple possible locations for the .env file
        # Get workspace directory from environment or derive it from current file path
        # This avoids hardcoding any workspace name
        if 'ROS_WORKSPACE' in os.environ:
            workspace_dir = os.environ['ROS_WORKSPACE']
        else:
            # Try to determine workspace from file path
            current_dir = Path(__file__).resolve().parent  # pi_patrol package python dir
            package_dir = current_dir.parent  # pi_patrol package dir
            src_dir = package_dir.parent  # src dir
            workspace_dir = src_dir.parent  # workspace dir
        package_name = 'pi_patrol'
        
        # Build paths dynamically - try multiple locations without hardcoding
        possible_paths = [
            # Current directory
            Path.cwd() / '.env',
            # Source directory (current package)
            Path(os.path.dirname(os.path.dirname(__file__))) / '.env',
            # Package directory
            Path(current_dir.parent) / '.env',
            # Workspace root
            Path(workspace_dir) / '.env',
            # Package directory in workspace
            Path(workspace_dir) / 'src' / package_name / '.env',
            # Home directory
            Path.home() / '.env',
            # ROS_HOME directory if set
            Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros'))) / '.env'
        ]
        
        env_loaded = False
        for dotenv_path in possible_paths:
            self.get_logger().info(f'Looking for .env file at: {dotenv_path.absolute()}')
            if dotenv_path.exists():
                load_dotenv(dotenv_path)
                self.get_logger().info(f'Found and loaded .env file at: {dotenv_path.absolute()}')
                env_loaded = True
                break
                
        if not env_loaded:
            self.get_logger().error('Could not find .env file in any of the expected locations')
        
        # Safely log the loaded values
        bot_token = os.getenv("TELEGRAM_BOT_TOKEN")
        chat_id = os.getenv("TELEGRAM_CHAT_ID")
        
        if bot_token:
            self.get_logger().info(f'Loaded bot token: {bot_token[:5]}... (truncated)')
        else:
            self.get_logger().error('Failed to load TELEGRAM_BOT_TOKEN from .env')
            
        if chat_id:
            self.get_logger().info(f'Loaded chat ID: {chat_id}')
        else:
            self.get_logger().error('Failed to load TELEGRAM_CHAT_ID from .env')
        
        # Telegram configuration
        self.bot_token = os.getenv('TELEGRAM_BOT_TOKEN')
        self.chat_id = os.getenv('TELEGRAM_CHAT_ID')
        
        # Check if credentials are properly set
        if not self.bot_token or self.bot_token == 'your_bot_token_here' or self.bot_token == 'your_telegram_bot_token_here':
            self.get_logger().error('Telegram bot token not set in .env file')
        if not self.chat_id or self.chat_id == 'your_chat_id_here' or self.chat_id == 'your_telegram_chat_id_here':
            self.get_logger().error('Telegram chat ID not set in .env file')
            
        self.telegram_api_url = f"https://api.telegram.org/bot{self.bot_token}"
        
        # ROS2 setup
        self.alert_subscription = self.create_subscription(
            String, '/intruder_alert', self.alert_callback, 10)
        
        # Notification settings
        # Use environment variable for recordings directory if set, otherwise use default
        self.recordings_dir = os.environ.get('PI_PATROL_RECORDINGS_DIR', os.path.expanduser("~/tracking_recordings"))
        self.last_notification_time = 0
        self.notification_cooldown = 30  # seconds between notifications
        
        self.get_logger().info('Telegram notifier initialized')
        
        # Test connection
        self.test_connection()
    
    def test_connection(self):
        """Test Telegram bot connection"""
        try:
            url = f"{self.telegram_api_url}/getMe"
            response = requests.get(url, timeout=10)
            if response.status_code == 200:
                bot_info = response.json()
                self.get_logger().info(f'Connected to Telegram bot: {bot_info["result"]["username"]}')
            else:
                self.get_logger().error('Failed to connect to Telegram bot')
        except Exception as e:
            self.get_logger().error(f'Telegram connection test failed: {e}')
    
    def send_message(self, message):
        """Send text message to Telegram"""
        try:
            url = f"{self.telegram_api_url}/sendMessage"
            data = {
                'chat_id': self.chat_id,
                'text': message,
                'parse_mode': 'HTML'
            }
            response = requests.post(url, data=data, timeout=10)
            if response.status_code == 200:
                self.get_logger().info('Telegram message sent successfully')
                return True
            else:
                self.get_logger().error(f'Failed to send Telegram message: {response.text}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error sending Telegram message: {e}')
            return False
    
    def send_video(self, video_path, caption=""):
        """Send video to Telegram"""
        try:
            if not os.path.exists(video_path):
                self.get_logger().warn(f'Video file not found: {video_path}')
                return False
            
            url = f"{self.telegram_api_url}/sendVideo"
            with open(video_path, 'rb') as video_file:
                files = {'video': video_file}
                data = {
                    'chat_id': self.chat_id,
                    'caption': caption
                }
                response = requests.post(url, files=files, data=data, timeout=30)
            
            if response.status_code == 200:
                self.get_logger().info(f'Video sent to Telegram: {video_path}')
                return True
            else:
                self.get_logger().error(f'Failed to send video: {response.text}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error sending video: {e}')
            return False
    
    def alert_callback(self, msg):
        """Handle intruder alerts"""
        current_time = datetime.now()
        
        # Check cooldown to avoid spam
        if current_time.timestamp() - self.last_notification_time < self.notification_cooldown:
            self.get_logger().info('Notification skipped due to cooldown')
            return
        
        if "INTRUDER_DETECTED" in msg.data:
            self.last_notification_time = current_time.timestamp()
            
            # Create alert message
            timestamp = current_time.strftime("%Y-%m-%d %H:%M:%S")
            alert_message = f"""
🚨 <b>INTRUDER ALERT</b> 🚨

📅 Time: {timestamp}
📍 Location: Pi Patrol Robot
🎯 Detection: {msg.data}

🎥 Recording started automatically...
            """
            
            # Send immediate notification
            if self.send_message(alert_message):
                self.get_logger().info('Intruder alert sent to Telegram')
            
            # Schedule video sending after recording completes
            # (We'll wait a bit for the recording to finish)
            self.create_timer(15.0, lambda: self.send_latest_video(timestamp))
    
    def is_valid_video(self, video_path):
        """Check if video is valid (not empty and has content)"""
        try:
            # Check if file exists and has size > 10KB (to avoid empty files)
            if not os.path.exists(video_path):
                return False
                
            file_size = os.path.getsize(video_path)
            if file_size < 10240:  # 10KB minimum
                self.get_logger().warn(f'Video file too small ({file_size} bytes), likely blank')
                return False
                
            # Additional check using ffprobe to get video duration
            import subprocess
            cmd = [
                'ffprobe', 
                '-v', 'error', 
                '-show_entries', 'format=duration', 
                '-of', 'default=noprint_wrappers=1:nokey=1', 
                video_path
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0 and result.stdout.strip():
                duration = float(result.stdout.strip())
                self.get_logger().info(f'Video duration: {duration:.2f} seconds')
                
                # Check if video is too short (invalid) or too long (not properly trimmed)
                if duration < 1.0:  # Less than 1 second is likely invalid
                    self.get_logger().warn(f'Video too short ({duration:.2f}s), skipping')
                    return False
                    
                # Allow a small margin of error (6.5-7.5 seconds)
                if abs(duration - 7.0) > 0.5:
                    self.get_logger().warn(f'Video duration ({duration:.2f}s) not within expected range (6.5-7.5s), but sending anyway')
                    
                return True
            else:
                self.get_logger().warn(f'Could not determine video duration: {result.stderr}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error validating video: {e}')
            return False
            
    def send_latest_video(self, timestamp):
        """Send the latest recorded video"""
        try:
            # Find the most recent video file
            video_files = [f for f in os.listdir(self.recordings_dir) if f.endswith('.mp4')]
            if video_files:
                latest_video = max(video_files, key=lambda f: os.path.getctime(
                    os.path.join(self.recordings_dir, f)))
                video_path = os.path.join(self.recordings_dir, latest_video)
                
                # Validate video before sending
                if self.is_valid_video(video_path):
                    caption = f"🎥 Intruder recording from {timestamp}"
                    self.send_video(video_path, caption)
                else:
                    self.get_logger().warn(f'Skipping invalid or blank video: {video_path}')
                    self.send_message(f"⚠️ Alert: Motion detected, but recording failed or was too short.")
            else:
                self.get_logger().warn('No video files found to send')
        except Exception as e:
            self.get_logger().error(f'Error sending latest video: {e}')

def main():
    rclpy.init()
    node = TelegramNotifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()