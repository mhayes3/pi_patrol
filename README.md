# Pi Patrol


> 🚧 Under construction 🚧 
>
>   TODO:
> - [ ] Add node for motor controls
> - [ ] Add missing requirements.txt
> - [ ] Fix api endpoint for retreiving recorded video clips


## What is this?

A ROS 2 package for a security robot that uses a camera and YOLO for intruder detection and tracking.

## Features

- **Intruder Detection:** Uses a camera and YOLOv8 to detect people and pets.
- **Video Recording:** Automatically records video when an intruder is detected.
- **Telegram Notifications:** Sends alerts with video clips to a specified Telegram chat.

## Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Jazzy:** Follow the official installation guide: [docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## Setup

### 1. Install System Dependencies

First, install the necessary ROS 2 packages and system tools:

```bash
sudo apt update
sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-image-transport python3-pip python3.12-venv ffmpeg
```

### 2. (Optional) Create a Python virtual environment

```bash
python3 -m venv ~/ros_ml_env
source ~/ros_ml_env/bin/activate
pip install --upgrade pip
```

### 3. Install Python Dependencies

Install the necessary Python libraries using the provided `requirements.txt` file.

```bash
pip install -r requirements.txt
```

> **Note:** This project requires specific versions of `numpy` (< 2.0) and `opencv-python` (< 4.9) to be compatible with the version of `cv_bridge` used in ROS 2 Jazzy. The `requirements.txt` file enforces these versions to prevent potential build or runtime errors.

### 4. Configure Telegram and .env

Set up a Telegram bot and add your credentials:

- Create a bot via @BotFather and copy the bot token
- Get your chat ID via @userinfobot
- Create the `.env` file (use the example if available):

```bash
cp src/pi_patrol/.env.example src/pi_patrol/.env  # if the example exists
```

Add these to `src/pi_patrol/.env`:

```
TELEGRAM_BOT_TOKEN=your_bot_token_here
TELEGRAM_CHAT_ID=your_chat_id_here
```

### 5. Build the ROS 2 Workspace

From the root of your project, build the workspace:

```bash
colcon build
```

## Usage

```bash
source install/setup.bash
ros2 launch pi_patrol pi_patrol.launch.py
```

To enable detection you'll need to arm the robot via the /mode endpoint and set the mode to armed.


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
