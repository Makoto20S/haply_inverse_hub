# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Bilateral teleoperation master-side controller for a **Haply Inverse3 + VerseGrip** haptic device, commanding a **Franka FR3** robot arm over ROS 2 DDS (WiFi). The master reads device position/orientation/buttons, maps coordinates to robot target poses, and renders force feedback from the slave's end-effector forces back to the operator's hand.

Language: Python 3.12. All code is in Chinese-commented Python; comments and logs are primarily in Chinese.

## Build & Run

This project runs inside Docker with ROS 2 Jazzy. The host must have the **Haply Desktop** app running (WebSocket at `ws://localhost:10001`).

```bash
# Build image and start container
./create_image.sh            # runs: docker compose up -d --build

# Enter container
docker exec -it haply_inverse bash

# Inside container: build the custom ROS 2 message package
source /opt/ros/jazzy/setup.bash
cd /HAPLY_INVERSE/ros_ws && colcon build --packages-select teleop_msgs
source /HAPLY_INVERSE/ros_ws/install/setup.bash

# Run the main node
ros2 run teleop_nodes device_node

# Or use the launch file
ros2 launch teleop_nodes teleop_launch.py
```

For cross-machine communication with the Franka workstation, set CycloneDDS env vars before running:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///opt/xr/config/cyclone_uri/multi_machine.cyclonedds.xml
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

VS Code Dev Container is also configured (`.devcontainer/devcontainer.json`); use "Reopen in Container".

## Architecture

### Data Flow

```
Inverse3+VerseGrip <--WebSocket--> inverse_comm.py (Device)
                                        |
                                  device_wrapper.py (DeviceWrapper)
                                        |
                                  device_node.py (ROS 2 DeviceNode)
                                    /         \
                          Publishes:           Subscribes:
                          /device/pose         /end_effector_force
                          /gripper_command     /pose_echo
                          /device/force_z
                                    |
                            ROS 2 DDS (WiFi)
                                    |
                          Franka FR3 workstation
```

### Key Modules

- **`device_node.py`** (root) — ROS 2 node. 100 Hz timer reads device state, applies coordinate mapping (device frame -> robot base frame via `R_DEV2ROB` rotation matrix), publishes `PoseStamped`. Subscribes to slave force, computes impedance+sensor force blend, contact pulse detection, and variable stiffness, then drives force feedback via `DeviceWrapper.set_external_force()`. ~1400 lines, the main integration point.

- **`Inverse_controller/inverse_comm.py`** — WebSocket client (`Device` class). Runs an asyncio event loop in a background thread, exchanges JSON messages with the Haply Desktop API via `orjson`. Receives position/velocity/buttons/orientation at ~1kHz, sends force commands back each cycle.

- **`Inverse_controller/device_wrapper.py`** — High-level wrapper (`DeviceWrapper` + `DeviceState` dataclass). Adds quaternion continuity fix, button edge detection (pressed/released), xyzw<->wxyz conversion, staleness-based connection status. Downstream code uses this, not `Device` directly.

- **`Inverse_controller/force_controller_1.py`** — `ForceFeedbackCalculator`. Virtual wall/floor force computation (currently disabled — walls return zero, only external force passthrough + low-pass filter + force limiting active). Loaded by `inverse_comm.py`.

- **`Inverse_controller/config_manager.py`** — Loads TOML configs from `Inverse_controller/config/`. Supports `leader.toml` (master) and `follower.toml` (slave) modes. Provides dot-path access: `get_value("network.uri")`.

- **`Inverse_controller/transformation.py`** — Quaternion utilities: `fix_quat_continuity` (sign-flip correction) and `quat_to_axes_xyzw` (format conversion).

### ROS 2 Packages

- **`teleop_msgs`** — Custom message: `DeviceState.msg` (position, velocity, orientation, buttons, connection status).
- **`teleop_nodes`** — Python package containing `device_node.py` (installed copy) and launch/config files. Entry point: `teleop_nodes.device_node:main`.

### Coordinate Frames

Device-to-robot mapping uses a 90-degree Z rotation: `R_DEV2ROB = [[0,-1,0],[1,0,0],[0,0,1]]`. Device X->Robot -Y, Device Y->Robot X, Z unchanged.

### Configuration

- Device connection and force feedback params: `Inverse_controller/config/leader.toml`
- ROS 2 node params (position scale, force gains, impedance K/C, dead zone, contact detection): declared in `device_node.py`, overridable via `--ros-args -p param:=value` or YAML file.

### Threading Model

`inverse_comm.py` runs an asyncio WebSocket loop in a daemon thread (started by `DeviceWrapper.start()`). The ROS 2 node runs in the main thread with a 100 Hz timer. Shared state is protected by `threading.Lock`.

## Dependencies

Python: `numpy`, `websockets`, `orjson`, `tomlkit` (see `requirements.txt`).
System: ROS 2 Jazzy, `rmw_cyclonedds_cpp`, Haply Desktop API.
