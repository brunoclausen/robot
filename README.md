#!/bin/bash
set -euo pipefail

LOG_FILE="install_log.txt"

# Helper functions
log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

error_exit() {
    log "ERROR: $1"
    exit 1
}

check_command() {
    command -v "$1" >/dev/null 2>&1 || error_exit "$1 is not installed. Please install it and rerun the script."
}

check_python_package() {
    python3 -c "import $1" >/dev/null 2>&1 || error_exit "Python package $1 is not installed. Please install it and rerun the script."
}

cleanup() {
    log "Cleaning up..."
    # Add any cleanup tasks here if needed
}
trap cleanup EXIT

# Clear log file
> "$LOG_FILE"

# Afsnit 0: Preliminary Checks
log "Checking required commands..."
check_command "sudo"
check_command "apt-get"
check_command "rosdep"
check_command "pip3"
check_command "git"
check_command "cmake"

log "Checking required Python packages..."
check_python_package "paho.mqtt.client"
check_python_package "RPi.GPIO"

# Ask for confirmation
log "This script will install several packages, modify system files, and update existing packages."
read -p "Do you want to continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    log "User chose not to continue. Exiting..."
    exit 0
fi

# Afsnit 1: System Preparation and Update
log "Updating system packages..."
sudo apt-get update -y | tee -a "$LOG_FILE" || error_exit "System update failed."
sudo apt-get upgrade -y | tee -a "$LOG_FILE" || error_exit "System upgrade failed."
sudo apt-get dist-upgrade -y | tee -a "$LOG_FILE" || error_exit "System dist-upgrade failed."

# Afsnit 2: Update Python Packages
log "Updating Python packages..."
pip3 list --outdated --format=freeze | grep -v '^\-e' | cut -d = -f 1 | xargs -n1 pip3 install -U | tee -a "$LOG_FILE" || error_exit "Python packages update failed."

# Afsnit 3: Update ROS Packages
log "Updating ROS packages..."
sudo apt-get update -y | tee -a "$LOG_FILE" || error_exit "ROS package update failed."
sudo apt-get upgrade -y ros-noetic-* | tee -a "$LOG_FILE" || error_exit "ROS packages upgrade failed."

# Afsnit 4: Installation of ROS Noetic
log "Installing ROS Noetic..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 | tee -a "$LOG_FILE" || error_exit "Failed to add ROS GPG key."
sudo apt-get update -y | tee -a "$LOG_FILE" || error_exit "ROS package update failed."
sudo apt-get install -y ros-noetic-ros-base | tee -a "$LOG_FILE" || error_exit "Failed to install ROS Noetic."
sudo rosdep init | tee -a "$LOG_FILE" || error_exit "rosdep init failed."
rosdep update | tee -a "$LOG_FILE" || error_exit "rosdep update failed."
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash

# Afsnit 5: Setup of Catkin Workspace
log "Setting up Catkin workspace..."
if [ -d "$HOME/catkin_ws/src" ]; then
    log "Catkin workspace already exists. Skipping workspace creation."
else
    mkdir -p "$HOME/catkin_ws/src" || error_exit "Failed to create Catkin workspace directory."
fi
cd "$HOME/catkin_ws/"
catkin_make | tee -a "$LOG_FILE" || error_exit "Failed to build Catkin workspace."
source devel/setup.bash || error_exit "Failed to source Catkin workspace."

# Afsnit 6: Installation of TensorFlow Lite Runtime
log "Installing TensorFlow Lite Runtime..."
pip3 install tflite-runtime | tee -a "$LOG_FILE" || error_exit "Failed to install TensorFlow Lite Runtime."

# Afsnit 7: Installation and Setup of Kinect Drivers
log "Installing Kinect drivers and necessary libraries..."
sudo apt-get install -y build-essential cmake git libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev libopencv-dev | tee -a "$LOG_FILE" || error_exit "Failed to install Kinect dependencies."

# Fetching and building libfreenect
log "Fetching and building libfreenect..."
cd "$HOME"
git clone https://github.com/OpenKinect/libfreenect.git | tee -a "$LOG_FILE" || error_exit "Failed to clone libfreenect repository."
cd libfreenect
mkdir build || error_exit "Failed to create build directory."
cd build
cmake .. | tee -a "$LOG_FILE" || error_exit "cmake configuration failed."
make | tee -a "$LOG_FILE" || error_exit "Failed to build libfreenect."
sudo make install | tee -a "$LOG_FILE" || error_exit "Failed to install libfreenect."
sudo ldconfig /usr/local/lib64/ | tee -a "$LOG_FILE" || error_exit "Failed to configure dynamic linker."

# Afsnit 8: Installation of MQTT Broker (Mosquitto)
log "Installing MQTT Broker..."
sudo apt-get install -y mosquitto mosquitto-clients | tee -a "$LOG_FILE" || error_exit "Failed to install Mosquitto."
sudo systemctl start mosquitto | tee -a "$LOG_FILE" || error_exit "Failed to start Mosquitto."
sudo systemctl enable mosquitto | tee -a "$LOG_FILE" || error_exit "Failed to enable Mosquitto."

# Afsnit 9: Installation of RPi.GPIO for Python
log "Installing RPi.GPIO library..."
pip3 install RPi.GPIO | tee -a "$LOG_FILE" || error_exit "Failed to install RPi.GPIO library."

# Afsnit 10: Creation of ROS Package and MQTT Control Node Script with GPIO Control
log "Creating ROS package and control node script..."
cd "$HOME/catkin_ws/src"
catkin_create_pkg my_robot_pkg std_msgs rospy roscpp geometry_msgs | tee -a "$LOG_FILE" || error_exit "Failed to create ROS package."

mkdir -p "$HOME/catkin_ws/src/my_robot_pkg/scripts" || error_exit "Failed to create scripts directory."
cat <<EOL > "$HOME/catkin_ws/src/my_robot_pkg/scripts/mqtt_control_node.py"
#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import RPi.GPIO as GPIO

# GPIO Pin Setup
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setwarnings(False)

# Define GPIO pins for motor control
PIN_FORWARD = 17
PIN_BACKWARD = 18
PIN_LEFT = 22
PIN_RIGHT = 23

# Setup GPIO pins as outputs
GPIO.setup(PIN_FORWARD, GPIO.OUT)
GPIO.setup(PIN_BACKWARD, GPIO.OUT)
GPIO.setup(PIN_LEFT, GPIO.OUT)
GPIO.setup(PIN_RIGHT, GPIO.OUT)

# Initialize MQTT parameters
mqtt_broker_ip = "192.168.1.5"
mqtt_topic = "robot/control"
mqtt_user = "mqtt_user"
mqtt_password = "mqtt"

# ROS node initialization
rospy.init_node('mqtt_control_node')
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
status_pub = rospy.Publisher('/robot_status', String, queue_size=10)

# MQTT connection and message handling
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Forbundet til MQTT broker med resultatkode " + str(rc))
    client.subscribe(mqtt_topic)

def on_message(client, userdata, msg):
    rospy.loginfo(f"Modtog MQTT besked: {msg.topic} {msg.payload}")
    command = msg.payload.decode('utf-8')
    process_command(command)

def process_command(command):
    twist = Twist()
    
    if command == "fremad":
        twist.linear.x = 0.5
        GPIO.output(PIN_FORWARD, GPIO.HIGH)
        GPIO.output(PIN_BACKWARD, GPIO.LOW)
        GPIO.output(PIN_LEFT, GPIO.LOW)
        GPIO.output(PIN_RIGHT, GPIO.LOW)
    elif command == "tilbage":
        twist.linear.x = -0.5
        GPIO.output(PIN_FORWARD, GPIO.LOW)
        GPIO.output(PIN_BACKWARD, GPIO.HIGH)
        GPIO.output(PIN_LEFT, GPIO.LOW)
        GPIO.output(PIN_RIGHT, GPIO.LOW)
    elif command == "venstre":
        twist.angular.z = 0.5
        GPIO.output(PIN_FORWARD, GPIO.LOW)
        GPIO.output(PIN_BACKWARD, GPIO.LOW)
        GPIO.output(PIN_LEFT, GPIO.HIGH)
        GPIO.output(PIN_RIGHT, GPIO.LOW)
    elif command == "højre":
        twist.angular.z = -0.5
        GPIO.output(PIN_FORWARD, GPIO.LOW)
        GPIO.output(PIN_BACKWARD, GPIO.LOW)
        GPIO.output(PIN_LEFT, GPIO.LOW)
        GPIO.output(PIN_RIGHT, GPIO.HIGH)
    elif command == "stop":
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        GPIO.output(PIN_FORWARD, GPIO.LOW)
        GPIO.output(PIN_BACKWARD, GPIO.LOW)
        GPIO.output(PIN_LEFT, GPIO.LOW)
        GPIO.output(PIN_RIGHT, GPIO.LOW)
    else:
        rospy.loginfo("Ugyldig kommando")
        GPIO.output(PIN_FORWARD, GPIO.LOW)
        GPIO.output(PIN_BACKWARD, GPIO.LOW)
        GPIO.output(PIN_LEFT, GPIO.LOW)
        GPIO.output(PIN_RIGHT, GPIO.LOW)

    cmd_vel_pub.publish(twist)
    status_pub.publish(f"Udførte kommando: {command}")

# Initialize MQTT client
mqtt_client = mqtt.Client()
mqtt_client.username_pw_set(mqtt_user, mqtt_password)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(mqtt_broker_ip, 1883, 60)

mqtt_client.loop_start()
rospy.spin()

# Cleanup GPIO on exit
GPIO.cleanup()
EOL

chmod +x "$HOME/catkin_ws/src/my_robot_pkg/scripts/mqtt_control_node.py" || error_exit "Failed to make mqtt_control_node.py executable."

# Afsnit 11: Creation of AI Camera Script with Blue Line Following
log "Creating AI camera script with blue line following functionality..."
cat <<EOL > "$HOME/catkin_ws/src/my_robot_pkg/scripts/ai_camera_node.py"
#!/usr/bin/env python3
import cv2
import numpy as np
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Initialize ROS node
rospy.init_node('ai_camera_node')
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
status_pub = rospy.Publisher('/camera_status', String, queue_size=10)

# Initialize MQTT client
mqtt_client = mqtt.Client()
mqtt_client.connect("192.168.1.5", 1883, 60)

# Initialize camera
cap = cv2.VideoCapture(0)

# Define the color range for detecting blue
lower_blue = np.array([100, 150, 0])
upper_blue = np.array([140, 255, 255])

def process_frame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert to HSV color space
    mask = cv2.inRange(hsv, lower_blue, upper_blue)  # Create mask for blue color
    return mask

def detect_line(mask):
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # Find the largest contour which is likely the blue line
        largest_contour = max(contours, key=cv2.contourArea)
        # Get the moments to calculate the centroid of the contour
        M = cv2.moments(largest_contour)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            return cx
    return None

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        rospy.loginfo("Failed to capture image")
        continue

    # Process the frame to detect the blue line
    mask = process_frame(frame)
    cx = detect_line(mask)

    twist = Twist()

    if cx is not None:
        # Determine action based on the position of the centroid
        frame_center = frame.shape[1] // 2
        if cx < frame_center - 50:  # Blue line is to the left
            twist.angular.z = 0.3  # Turn left
            rospy.loginfo("Turning left")
            mqtt_client.publish("robot/camera", "Turning left")
        elif cx > frame_center + 50:  # Blue line is to the right
            twist.angular.z = -0.3  # Turn right
            rospy.loginfo("Turning right")
            mqtt_client.publish("robot/camera", "Turning right")
        else:
            twist.linear.x = 0.5  # Move forward
            rospy.loginfo("Moving forward")
            mqtt_client.publish("robot/camera", "Moving forward")
    else:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        rospy.loginfo("Blue line not detected, stopping")
        mqtt_client.publish("robot/camera", "Stopping")

    cmd_vel_pub.publish(twist)
    status_pub.publish("Line following active")

cap.release()
EOL

chmod +x "$HOME/catkin_ws/src/my_robot_pkg/scripts/ai_camera_node.py" || error_exit "Failed to make ai_camera_node.py executable."

# Afsnit 12: Create a Launch File
log "Creating launch file..."
mkdir -p "$HOME/catkin_ws/src/my_robot_pkg/launch" || error_exit "Failed to create launch directory."
cat <<EOL > "$HOME/catkin_ws/src/my_robot_pkg/launch/robot_launch.launch"
<launch>
    <node name="mqtt_control_node" pkg="my_robot_pkg" type="mqtt_control_node.py" output="screen"/>
    <node name="ai_camera_node" pkg="my_robot_pkg" type="ai_camera_node.py" output="screen"/>
</launch>
EOL

# Afsnit 13: Build and Source Workspace
log "Building and setting up ROS workspace..."
cd "$HOME/catkin_ws"
catkin_make | tee -a "$LOG_FILE" || error_exit "Failed to build Catkin workspace."
source devel/setup.bash || error_exit "Failed to source Catkin workspace."

# Afsnit 14: Test Script to Check Installation
log "Creating and running test script..."
cat <<EOL > "$HOME/catkin_ws/src/my_robot_pkg/scripts/test_installation.sh"
#!/bin/bash
set -euo pipefail

LOG_FILE="test_log.txt"

log() {
    echo "\$(date '+%Y-%m-%d %H:%M:%S') - \$1" | tee -a \$LOG_FILE
}

error_exit() {
    log "ERROR: \$1"
    exit 1
}

# Clear log file
> \$LOG_FILE

log "Starting installation tests..."

# Test ROS environment
log "Testing ROS environment..."
source /opt/ros/noetic/setup.bash || error_exit "ROS setup.bash failed."
source "$HOME/catkin_ws/devel/setup.bash" || error_exit "Catkin workspace setup.bash failed."

# Test ROS nodes
log "Testing ROS nodes..."
roslaunch my_robot_pkg robot_launch.launch --screen & # Start the nodes in the background
ROS_PID=\$!
sleep 10 # Wait for nodes to initialize

if ! ps -p \$ROS_PID > /dev/null; then
    error_exit "ROS nodes did not start successfully."
else
    log "ROS nodes started successfully."
fi
kill \$ROS_PID # Stop the ROS nodes

# Test MQTT connection
log "Testing MQTT connection..."
MQTT_BROKER="192.168.1.5"
mosquitto_pub -h \$MQTT_BROKER -t "robot/test" -m "test message" || error_exit "Failed to publish test message to MQTT broker."
log "MQTT connection test passed."

# Test GPIO functionality (Mock test)
log "Testing GPIO setup (mock test)..."
python3 - <<EOF
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.output(17, GPIO.HIGH)
GPIO.output(17, GPIO.LOW)
GPIO.cleanup()
EOF
log "GPIO functionality test passed."

log "All tests passed successfully!"
EOL

chmod +x "$HOME/catkin_ws/src/my_robot_pkg/scripts/test_installation.sh" || error_exit "Failed to make test_installation.sh executable."
"$HOME/catkin_ws/src/my_robot_pkg/scripts/test_installation.sh" | tee -a "$LOG_FILE" || error_exit "Installation test script failed."

log "Installation, update, and testing completed successfully!"
