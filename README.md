# tak_cot_bridge

ROS 2 package that subscribes to MAVROS topics and publishes Cursor-on-Target (CoT) events to FreeTAKServer.

## Features

- Compatible with ROS 2 Humble and Jazzy.
- Listens to GPS and state data from MAVROS (CubePilot, PX4).
- Sends real-time CoT events over UDP to local or cloud FreeTAKServer.
- Easily configurable IP and port via YAML file.

## Nodes

### `cot_publisher_node.py`

- Subscribes to `/mavros/global_position/global` (NavSatFix) and other MAVROS telemetry topics.
- Constructs CoT XML events.
- Publishes events via UDP to configured TAKServer endpoint.

## Usage

```bash
ros2 launch tak_cot_bridge cot_bridge.launch.py