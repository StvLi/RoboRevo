# teleop_infer_infra

Teleoperation and model inference infrastructure package for FR3 robot.

## Overview

This package provides infrastructure for teleoperation and model inference deployment on the FR3 robot. It is designed to be independent of the `franka_ros` package while providing custom message types and utilities for teleoperation and inference tasks.

## Features

- **Custom Messages**: Defines `EquilibriumPose` message type for Cartesian impedance control equilibrium positions
- **Teleoperation Support**: Infrastructure for teleoperation interfaces
- **Inference Deployment**: Utilities for deploying and running machine learning models

## Package Structure

```
teleop_infer_infra/
├── msg/                    # Custom message definitions
│   └── EquilibriumPose.msg # Equilibrium pose message
├── scripts/                # Executable Python scripts
├── src/                    # Python source code
│   └── teleop_infer_infra/ # Python package
├── launch/                 # Launch files
├── config/                 # Configuration files
├── CMakeLists.txt
├── package.xml
└── setup.py
```

## Messages

### EquilibriumPose

Custom message type for equilibrium pose in Cartesian impedance control.

**Fields**:
- `std_msgs/Header header` - Standard ROS header with timestamp and frame_id
- `geometry_msgs/Pose pose` - Position and orientation of the equilibrium pose

This message is independent from `franka_ros` but compatible with the equilibrium pose format used by Cartesian impedance controllers.

## Dependencies

- `rospy` - Python ROS client library
- `std_msgs` - Standard ROS messages
- `geometry_msgs` - Geometry messages
- `sensor_msgs` - Sensor messages
- `message_generation` - For building custom messages
- `message_runtime` - For running with custom messages

## Building

```bash
cd ~/fr3_ros1_infra
catkin_make
source devel/setup.bash
```

## Usage

This package is designed as infrastructure for teleoperation and inference systems. Implementation details will be added in subsequent development.

## License

MIT

