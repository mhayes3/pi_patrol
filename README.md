# Pi Patrol

A ROS 2 package for Raspberry Pi-based security monitoring with motion detection, video recording, and Telegram notifications.

## Overview

Pi Patrol is a comprehensive security monitoring system built on ROS 2 that uses a Raspberry Pi camera to detect motion, record video evidence, and send notifications via Telegram. When an intruder is detected, the system records a 7-second video clip (including pre-motion buffer) and sends it directly to your Telegram account.

## System Architecture

Pi Patrol consists of four main nodes:

1. **Camera Node**: Captures frames from the Raspberry Pi camera and publishes them to the `/camera/image_raw` topic
2. **Detection Node**: Subscribes to camera frames, runs YOLO object detection, and publishes intruder alerts
3. **Recorder Node**: Maintains a buffer of recent frames and records videos when an intruder is detected
4. **Telegram Notifier Node**: Sends notifications and video evidence to Telegram when alerted

## Prerequisites

- Raspberry Pi (tested on Raspberry Pi 4/5)
- Raspberry Pi Camera Module
- ROS 2 (tested on Humble)
- Python 3.8+
- OpenCV
- FFmpeg
- python-dotenv
- Telegram Bot (see setup instructions below)

## Installation

### 1. Clone the repository into your ROS 2 workspace

```bash
cd ~/your_ros2_workspace/src
git clone https://github.com/mhayes3/pi_patrol.git
```

### 2. Install dependencies

```bash
sudo apt update
sudo apt install -y ffmpeg python3-opencv python3-pip
pip3 install python-dotenv requests
```

### 3. Create your .env file

Create a `.env` file in the workspace root or in the package directory:

```bash
cd ~/your_ros2_workspace
cp src/pi_patrol/.env.example .env
```

Edit the `.env` file to add your Telegram credentials:

```
TELEGRAM_BOT_TOKEN=your_bot_token_here
TELEGRAM_CHAT_ID=your_chat_id_here
```

### 4. Build the package

```bash
cd ~/your_ros2_workspace
colcon build --packages-select pi_patrol
source install/setup.bash
```

## Telegram Bot Setup

1. Create a new Telegram bot by messaging [@BotFather](https://t.me/botfather) on Telegram
2. Follow the instructions to create a new bot and receive your bot token
3. Start a conversation with your bot
4. Get your chat ID by messaging [@userinfobot](https://t.me/userinfobot)
5. Update your `.env` file with the bot token and chat ID

## Usage

### Launch all nodes

```bash
ros2 launch pi_patrol pi_patrol.launch.py
```

### Run individual nodes

```bash
# Camera node
ros2 run pi_patrol camera_node

# Detection node
ros2 run pi_patrol detection_node

# Recorder node
ros2 run pi_patrol recorder_node

# Telegram notifier node
ros2 run pi_patrol telegram_notifier_node
```

### Manually trigger an alert

```bash
ros2 topic pub --once /intruder_alert std_msgs/msg/String "data: INTRUDER_DETECTED"
```

## Configuration

### Environment Variables

- `TELEGRAM_BOT_TOKEN`: Your Telegram bot token
- `TELEGRAM_CHAT_ID`: Your Telegram chat ID
- `PI_PATROL_RECORDINGS_DIR`: Directory to save recordings (default: ~/tracking_recordings)
- `ROS_WORKSPACE`: ROS workspace path (optional, auto-detected if not set)

### Node Parameters

Parameters can be adjusted in the launch file or when running individual nodes:

- Camera resolution and FPS
- Detection confidence threshold
- Recording buffer size and duration
- Notification cooldown period

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Acknowledgments

- ROS 2 Community
- OpenCV Project
- Ultralytics YOLO
- python-telegram-bot
