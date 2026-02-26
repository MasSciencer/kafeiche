# Kafeiche Robot Software

## Overview

This repository contains the source code and configuration for the **Kafeiche**
three‑wheeled heavy‑load mobile robot. The workspace is organized into two ROS 2
packages:

* `kafeiche_description` – URDF/Xacro description, 3‑D models, and launch
  files.
* `kafeiche_drivers` – hardware interface and low‑level driver classes for
  motors and encoders.

A small utility under the `tool/` directory provides keyboard teleoperation via
rosbridge.

## Directory layout

```
.
├── README.md                 # this file
├── src/
│   ├── kafeiche_description
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── launch/
│   │   │   ├── main.launch.py   # standard robot launch
│   │   │   └── vr.launch.py     # includes rosbridge for remote/VR control
│   │   ├── meshes/              # 3‑D model files
│   │   └── urdf/
│   │       ├── kafeiche_base.xacro
│   │       └── kafeiche_control.xacro
│   ├── kafeiche_drivers
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── controller_hw_interface.xml
│   │   ├── config/
│   │   │   ├── controller.yaml
│   │   │   └── xbox_custom.yaml
│   │   ├── include/kafeiche_drivers/
│   │   │   ├── controller.hpp
│   │   │   ├── encoder.hpp
│   │   │   └── motor.hpp
│   │   └── src/
│   │       └── controller.cpp
│   └── web_interface
│       ├── main.py
│       └── static/index.html
├── tool/
│   └── keyboard_control.py     # rosbridge teleop helper
└── start.sh                    # helper script for development
```

## Building

Make sure a ROS 2 workspace has been sourced and that the `pigpiod_if2`
client library (and ideally the core `pigpio` library as well) are installed on
the host. A running `pigpiod` daemon is required for the former. Then:

```bash
cd <workspace>
colcon build --packages-select kafeiche_description kafeiche_drivers
```

## Running

* Launch the basic robot stack:
  ```bash
  ros2 launch kafeiche_description main.launch.py
  ```
* Start with rosbridge (for VR or remote control):
  ```bash
  ros2 launch kafeiche_description vr.launch.py
  ```
* Use the Python teleop utility:
  ```bash
  python3 tool/keyboard_control.py
  ```

## ROS 2 nodes and interfaces

* `kafeiche_drivers::DiffKfc` – custom `ros2_control` SystemInterface
  implementing differential-drive hardware.
* `motor.hpp`, `encoder.hpp` – low‑level classes managing pigpiod_if2 (daemon) 
  GPIO and SPI hardware.  **Motor interface now accepts and reports angular
  velocity in radians per second (rad/s)** to keep units consistent across the
  stack.
* Launch files configure `ros2_control_node`, `robot_state_publisher`, and the
  diff_drive controller.

## License

Specify project license here (e.g. BSD, Apache‑2.0).

