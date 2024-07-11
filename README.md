# BlueROV2 ROS2 Driver Barebones Interface
- Originally adapted from https://github.com/patrickelectric/bluerov_ros_playground
- Modularized and re-written for ROS2. 
- Barebones driver only, modular and user-readable to add your own snippets. 

## Requirements
- ROS2
- Pymavlink 

## Topics
### Publishers:
- /bluerov2/altitude
- /bluerov2/battery
- /bluerov2/bottle_pressure
- /bluerov2/imu
- /bluerov2/odometry
- /bluerov2/raw

### Subscribers
- /bluerov2/heartbeat
- /bluerov2/set_pwm

### Installation ###
1. Clone this project in your colcon_ws/src.
   - `$ git clone https://github.com/bvibhav/bluerov2_interface`
2. Go back to your ROS Colcon workspace:
3. Build  it:
   - `colcon build --event-handlers console_direct+ --cmake-args --symlink-install --packages-select bluerov2_interface`
3. Reload your colcon workspace.

## Running ##
```
ros2 launch bluerov2_interface bluerov2_launch.xml
```