# ROS2 LIDAR Object Detector
This project demonstrates LIDAR-based object detection in a differential drive robot simulation using ROS 2.The package processes LaserScan data from the `/scan` topic by ignoring invalid ranges and values to extract the closest object distance directly in front of the robot and publishes the filtered distance to the `/closest_object_distance` topic for real-time monitoring and control applications.

Click [here](https://youtu.be/wR66sn73yjY) for the simulation video.

## System Requirements
- **Ubuntu**: 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Gazebo**: Ignition Garden

## Usage

### Prerequisites
Install required ROS 2 packages:
```bash
sudo apt install ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher \
                 ros-humble-xacro \
                 ros-humble-teleop-twist-keyboard \
                 ros-humble-ros-gz-sim \
                 ros-humble-ros-gz-bridge \
                 ros-humble-sensor-msgs
```

### Installation & Setup
1. **Create Workspace and Clone Repository**
```bash
mkdir -p my_ws/src && cd my_ws/src
git clone https://github.com/yashikasharma0301/ros2-lidar-object-detector.git
```

2. **Build the Workspace**
```bash
cd ..
colcon build
```

3. **Source the Workspace**
```bash
source install/setup.bash
```

### Running the Simulation

1. **Launch Robot with LIDAR in Gazebo**
```bash
ros2 launch ros2-lidar-object-detector myrobot.launch.py
```
<img width="1020" height="900" alt="image" src="https://github.com/user-attachments/assets/79334c0b-150d-43c4-8764-eb47e342b401" />

To enable LIDAR visualization:
- Click on the three dots on the upper right corner of your Gazebo window
- Search for **Visualize Lidar** from the menu and click on it
- Refresh the lists of topics and choose the '/scan' topic. Ensure that **Display Lidar Visualisation** is checked.

2. **Start the LIDAR Object Detection Node**

In a new terminal, source the workspace and run the scan filter:
```bash
cd my_ws
source install/setup.bash
ros2 run ros2-lidar-object-detector scan_filter
```

3. **Control the Robot**

In a new terminal, start keyboard teleoperation to test the detection:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Control Keys:**
- `i` - Move forward
- `k` - Stop
- `j` - Turn left  
- `l` - Turn right
- `u` - Move forward + turn left
- `o` - Move forward + turn right
- `m` - Move backward + turn left
- `.` - Move backward + turn right
- `,` - Move backward
<img width="1854" height="1056" alt="image" src="https://github.com/user-attachments/assets/f22c26f8-b646-4669-9b7a-49c361c99c2b" />


## Credits & References
**Robot Model**: This project uses a URDF model adapted from the TortoiseBot example in the [OSRF ROS Book](https://github.com/osrf/rosbook/blob/master/code/tortoisebot/tortoisebot.urdf). The original model has been modified for ROS 2 Humble and Gazebo Ignition integration with LIDAR sensor capabilities.

**Original Authors**: Open Source Robotics Foundation (OSRF)
