# micro-ROS-teleop-robot

This project enables control of a differential-drive robot using an ESP32 board connected over Wi-Fi and integrated with ROS 2 (Humble distribution) via micro-ROS.
The ESP32 receives movement commands (/cmd_vel) of turtlesim package from ROS2 and translates them into control signals for two 360° servo motors, representing the left and right wheels of the robot.

# Requirements

💻 Laptop with ROS 2 Humble installed

🔌 ESP32 board

⚙️ Two 360° servo motors

🔋 Breadboard / test board

🧵 Connecting wires

🤖 Robot chassis (mechanical design)

# Running

## Setting up micro-ROS Agent

On your ROS 2 Humble laptop, run the micro-ROS Agent:
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Ensure the ESP32 connects using the same Wi-Fi network, with the following setup line in your Arduino code:
```
set_microros_wifi_transports("SSID", "PASSWORD", "AGENT_IP", 8888);
```

where you should replace:

SSID → your Wi-Fi network name

PASSWORD → your Wi-Fi password

AGENT_IP → IP address of your laptop running the micro-ROS agent

## Running Teleoperation (Control)
To test robot motion using keyboard teleoperation:
```
ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/cmd_vel
```

Use arrow keys to send movement commands:

⬆️ Forward

⬇️ Backward

⬅️ Rotate Left

➡️ Rotate Right

🧩 enjoy with your robot movement now. 
