# MVP C2 Reporter ROS node

## Description
`mvp_c2_report` is running on the vehicle side. It reports vehicle status, including odometry, geoposition, roslaunch file, power port, waypoints, helm state, and controller state, and etc.. The information are encoded into DCCL message and published to `mvp_c2/reporter/dccl_msg_tx` topic.
Meanwhile, it subscribeds to `mvp_c2/reporter/dccl_msg_rx` topic. It parse all the messages coming from this topic and publish to other topics or call services created by vehicle controller and helm software.

## Published Topics

| Topic | Type | Description |
|------|------|-------------|
| `mvp_c2/reporter/dccl_msg_tx` | `std_msgs/ByteMultiArray` | Outgoing DCCL messages |
| `joy` | `sensor_msgs/Joy` | Joy message transmitted from the commander over the communication channels |
---

## Subscribed Topics

| Topic | Type | Description |
|------|------|-------------|
| `mvp_c2/reporter/dccl_msg_rx` | `std_msgs/ByteMultiArray` | Incoming DCCL messages|
| `local/odometry` | `nav_msgs/Odometry` | Odometry data that will be converted into DCCL message|
| `local/geopose` | `geographic_msgs/GeoPoseStamped` | Odometry in lat-lon-altitude that will be converted into DCCL message|
| `local/acomm_geopoint` | `geographic_msgs/GeoPointStamped` | Geographic position of the vehicle position obtained from topside USBL|
| `local/power_monitor` | `std_msgs/Float32MultiArray` | Voltage and current measurements from the power monitoring system |
| `local/computer_info` | `std_msgs/Float32MultiArray` | Computer status information such as CPU usage, temperature, and load |
| `local/altimeter` | `geometry_msgs/PointStamped` | Distance to bottom measurement from a DVL or altimeter|
| `local/feature_geo_points_sub` | `std_msgs/Float32MultiArray` | Geographic coordinates of detected features from LIDAR or sonar |
---

### Services
No service created from this node.

### Clients
| Service | Type | Description |
|--------|------|-------------|
| `controller/set` | `std_srvs/SetBool` | Set controller state |
| `controller/get_state` | `std_srvs/Trigger` | Get controller state  |
| `mvp_helm/change_state` | `mvp_msgs/ChangeState` | Change helm state|
| `mvp_helm/get_state` | `mvp_msgs/GetState` | Get current helm state and connected state |
| `mvp_helm/path` | `mvp_msgs/GetWaypoints` | Get a list of waypoints|
| `mvp_helm/set_waypoints` | `mvp_msgs/SendWaypoints` | Set waypoints of the path following behavior in helm (size limit set in DCCL proto)|
| `gpio_manager/get_power_status` | `std_srvs/Trigger` | Report the current Power port status |
| `gpio_manager/set_power/<gpio_devices>` | `std_srvs/SetBool` | Turn a power port on or off |
| `reset_datum` | `std_srvs/Trigger` | Reset the datum of the localization system |

## Parameters
- `local_id` (Default: `2`): ID of the reporter. The reporter will only response to the incoming DCCL message with matching remote_id.

- `remote_id` (Default: `1`): Remote id of the outgoing DCCL message.

- `dccl_rx_interval` (Default: `1.0`): Only the earliest DCCL message will be used if multiple DCCL message with the same ID has receiveid with in the time interval.

- `dccl_tx_interval` (Default: `1.0`): The timer (in seconds) for how often to call the following services and get `roslaunch` status.
    - `controller/get_state` 
    - `mvp_helm/get_state`
    - `mvp_helm/path`,
    -  `gpio_manager/get_power_status`

- `local_mvp_active` (Default: `True`): Indicates whether the local MVP system is active. (NOT USED)

- `service_wait_time` (Default: `0.2`): Time in seconds to wait for service responses.

- `helm_state_list` (Default: `['']`): List of available helm states. The state will be represented in `int` in the DCCL message. The sequence has to be the same as the commander.

- `launch_packages` (Default: `['']`): A list of ROS package names for the launch files. The names will be represented in `int` in the DCCL message. The sequence has to be the same as the commander.

- `launch_files` (Default: `['']`): A list of launch files without `.py.launch` extension. The names will be represented in `int` in the DCCL message. The sequence has to be the same as the commander. 

- `gpio_devices` (Default: `['']`): List of GPIO-controlled devices for power management. The names will be represented in `int` in the DCCL message. The sequence has to be the same as the commander. 