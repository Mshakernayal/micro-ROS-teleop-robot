# two_wheel_bot_ros2
URDF and simulation package for a simple two-wheel differential drive robot built for ROS 2 Jazzy. Includes RViz and Gazebo launch files for visualization, control, and simulation.

### Installation

### Clone the Repository:


cd ~/your_workspace_name/src 

git clone https://github.com/kowsik16/two_wheel_bot_ros2.git

### Install Dependencies:

sudo apt update && rosdep update

rosdep install --from-paths src --ignore-src -r -y


### Build the Package:

cd ~/your_workspace_name

colcon build --packages-select robot_description

source install/setup.bash


### Launch in RViz:

ros2 launch robot_description display.launch.py


### Launch in Gazebo:

ros2 launch robot_description gazebo.launch.py
