#!/bin/bash

# Save any local changes and pull latest updates
echo "Stashing changes and pulling from git..."
git stash
git pull

# Build the workspace
echo "Building with catkin_make..."
catkin_make

# Source the workspace
echo "Sourcing devel/setup.bash..."
source devel/setup.bash

# Make notspot_controller script executable
echo "Setting permissions for robot_controller_gazebo.py..."
roscd notspot_controller/scripts || { echo "roscd failed for notspot_controller/scripts"; exit 1; }
chmod +x robot_controller_gazebo.py

# Copy RoboticsUtilities to site-packages
echo "Copying RoboticsUtilities to site-packages..."
cp -r RoboticsUtilities ~/.local/lib/python3.8/site-packages

# Make notspot_joystick script executable
echo "Setting permissions for ramped_joystick.py..."
roscd notspot_joystick/scripts || { echo "roscd failed for notspot_joystick/scripts"; exit 1; }
chmod +x ramped_joystick.py

# Make notspot_navigation scripts executable
echo "Setting permissions for notspot_navigation/scripts..."
roscd notspot_navigation/scripts || { echo "roscd failed for notspot_navigation/scripts"; exit 1; }
chmod +x simple_odom_publisher.py cmd_vel_to_joints.py

echo "Done"

