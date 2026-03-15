# MVP C2 Commander ROS node

## Description
`mvp_c2_commander` is running on the topside or a master robot. It creates ros services and topics allowing users to start ROS launch files, change helm state, switch controller state, and update waypoints of a remote vehicle. The commands are encoded into DCCL message and published to `mvp_c2/commander/dccl_msg_tx` topic as a `ByteMultiArray` ROS message. Meanwhile, the commander subscribes to `mvp_c2_commander/dccl_msg_rx` topic to receive DCCL messages, then decode them and publish to different topics.


## Topic prefix (Maybe changed to a more convenient way)
`topic_prefix` is used to allow multiple commander running on the same machine under the same namespace
`topic_prefix` is added to different all topics and services `mvp_c2_commander/remote/id_<remote_id>`, except the DCCL rx and tx topics.
We will note the topic names with prefix using `<topic_prefix>`

## Published Topics

| Topic | Type | Description |
|------|------|-------------|
| `mvp_c2/commander/dccl_msg_tx` | `std_msgs/ByteMultiArray` | Outgoing DCCL messages |
| `<topic_prefix>/odometry` | `nav_msgs/Odometry` | Odometry from the remote vehicle |
| `<topic_prefix>/geopose` | `geographic_msgs/GeoPoseStamped` | Geopose of the remote vehicle |
| `<topic_prefix>/acomm/navsatfix` | `sensor_msgs/NavSatFix` | A NavSatFix of an acomm fix from remote vehicle|
| `<topic_prefix>/odometry/navsatfix` | `sensor_msgs/NavSatFix` | Republish Geopose of a remote vehicle in NavSatFix msg type |
| `<topic_prefix>/controller_state` | `std_msgs/Bool` | Controller state of the remote vehicle |
| `<topic_prefix>/helm/state` | `HelmState` | Current helm state of the remote vehicle|
| `<topic_prefix>/survey/geopath` | `Waypoints` | Programmed waypoints of the remote vehicle |
| `<topic_prefix>/roslaunch_state` | `std_msgs/Int16MultiArray` | ROS Launch process status of the remote vehicle |
| `<topic_prefix>/gpio_power_state` | `std_msgs/Int16MultiArray` | Power port status of the remote vehicle |
| `<topic_prefix>/power_info` | `std_msgs/Float32MultiArray` | Voltage and current of the remote vehicle |
| `<topic_prefix>/computer_info` | `std_msgs/Float32MultiArray` | Computer status of the remote vehicle |
| `<topic_prefix>/altimeter` | `geometry_msgs/PointStamped` | Altimeter reading of the remote vehicle |
| `<topic_prefix>/feature_points` | `std_msgs/Float32MultiArray` | Latitude, longitude and altitude of feature points detected by the remote vehicle |

---

## Subscribed Topics

| Topic | Type | Description |
|------|------|-------------|
| `mvp_c2/commander/dccl_msg_rx` | `std_msgs/ByteMultiArray` | Incoming DCCL messages|
| `joy` | `sensor_msgs/Joy` | Joystick commands from a Joy node|

---

### Services
| Service | Type | Description |
|--------|------|-------------|
| `<topic_prefix>/controller/set` | `std_srvs/SetBool` | Enable or disable the controller of the remote vehicle |
| `<topic_prefix>/mvp_helm/change_state` | `std_srvs/SetString` | Change the helm state of the remote vehicle |
| `<topic_prefix>/mvp_helm/set_waypoints` | `SendWaypoints` | Send a new set of waypoints to the remote vehicle helm |
| `<topic_prefix>/reset_datum` | `std_srvs/Trigger` | Reset the system geographic datum |
| `<topic_prefix>/roslaunch/<package>/<launch_file>` | `std_srvs/SetBool` | Start or stop a specific ROS launch file on the remote vehicle |
| `<topic_prefix>/gpio_manager/set_power/<device>` | `std_srvs/SetBool` | Turn power on or off for a specific GPIO controlled power port |

### Clients
No client created by this node


## Parameters
- `local_id` (Default: `1`): ID of the commander. The commander will only response to the incoming DCCL message with matching remote_id.

- `remote_id` (Default: `2`): Remote id of the outgoing DCCL message.

- `dccl_rx_interval` (Default: `1.0`): Only the earliest DCCL message will be used if multiple DCCL messages with the same ID has receiveid within the time interval.

- `dccl_tx_joy_interval` (Default: `0.1`): NOT USED

- `helm_state_list` (Default: `['']`): List of available helm states. The state will be represented in `int` in the DCCL message. The sequence has to be the same as the reporter.

- `launch_packages` (Default: `['']`): A list of ROS package names for the launch files. The names will be represented in `int` in the DCCL message. The sequence has to be the same as the commander.

- `launch_files` (Default: `['']`): A list of launch files without `.py.launch` extension. The names will be represented in `int` in the DCCL message. The sequence has to be the same as the commander. 

- `gpio_devices` (Default: `['']`): List of GPIO-controlled devices for power management. The names will be represented in `int` in the DCCL message. The sequence has to be the same as the commander. 