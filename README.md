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

### Parameters

The `acf.py` executable exposes the following parameters:

- `ip` -> the acf ip address
- `f_max` -> the maximum allowed force [N]
- `initial_force` -> the force generated when no contact is detected [N]
- `ramp_duration` -> the ramp time of the force once contact is detected [s]
- `payload` -> the mass attached to the acf [kg]
- `frequency` -> the polling frequency [Hz]
- `joint_name` -> the ACF joint name

### Polling mode

If the `frequency` parameter is set, commands will be set to the ACF at that rate. Received force commands will not immediately trigger sending of commands to the ACF in this mode. ACF telemetry will be received at the same rate.

If the parameter is set to zero, commands will only be sent to the ACF when a force command is received. Telemetry is only sent after each force command.

### Joint State Publisher

This node will also act as a joint state publisher when the `joint_name` parameter is set. This may be useful in conjunction with a robot state publisher.

### Services

The `ACF` node creates the following services:

- `/ACF/set_payload` (SetFloat) -> used to set the current payload [kg]
- `/ACF/set_f_zero` (SetFloat) -> used to set force generated when no contact is detected [N]
- `/ACF/set_t_ramp` (SetDuration) -> used to set the ramp time of the force once contact is detected [s]

### Publishers

The `ACF` node publishes on the following topic:

- `/ACF/telem` (ACFTelem) -> Contains return information from the ACF. Only published after receiving a force command.
- `/joint_state` (JointState) -> Publishes the current position of the ACF as a joint state. Only published if the `joint_name` parameter is set.

### Subscribers

The `ACF` node subscribes to the following topic:

- `/ACF/force` (Float32) -> Used to set the current output force of the ACF in Newtons. Telemetry information is sent after receiving a force command.
