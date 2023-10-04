# Ferrobotics ACF

## Overview

A ROS2 package for interfacting with a [Ferrobotics Active Contact Flange (ACF)](https://www.ferrobotics.com/en/services/products/active-contact-flange/) over TCP/IP Ethernet.

## Installation

### Create a workspace

```bash
mkdir -p ros2_ws/src
```

### Clone this repo

Clone the correct version for **Humble** from this repository inside your workspace src:

```bash
cd ros2_ws/src
git clone https://github.com/thettasch/ferrobotics_acf.git -b humble
cd ..
```

Finally, build all packages in the workspace:

```bash
colcon build --symlink-install
```

It may be necessary to make the python scripts executable:

```bash
chmod +x src/ferrobotics_acf/src/*.py
```

This command is only required if the `--symlink-install` flag is used.

## Usage

To run the acf interface:

```bash
ros2 run ferrobotics_acf acf.py
```

If there is an `executable not found` error, refer to the chmod command above.

To run the acf interface with an ACF with a non-default ip address:

```bash
ros2 run ferrobotics_acf acf.py --ros-args -p ip:=192.168.123.132
```
