# Introduction
**mvp_c2** is a python-based ROS2 package designed for marine vehicle control and command via multiple ways of communications, such as RF, WIFI, and acoustic modem.

**Data encoding/decoding** is based on `DCCL` library. Each encoded DCCL message is argumented with header `$$`, `XOR` checksum, and `\n` ending.


# mvp_c2_reporter
`mvp_c2_report` is running on the vehicle side. It reports vehicle status, including odometry, geoposition, roslaunch file, power port, waypoints, helm state, and controller state, and etc.. The information are encoded into DCCL message and published to `mvp_c2/reporter/dccl_msg_tx` topic as a `ByteMultiArray` ROS message.

### Published topics

### Subscribed topics

### Services

### Clients

### Parameters


# mvp_c2_commander
**mvp_c2_commander** is running on the topside or a master robot. It creates ros services and topics allowing users to start ROS launch files, change helm state, switch controller state, and update waypoints of a remote vehicle. The commands are encoded into DCCL message and published to `mvp_c2/commander/dccl_msg_tx` topic as a `ByteMultiArray` ROS message.

### Published topics

### Subscribed topics

### Services

### Clients

### Parameters


# mvp_c2_traffic_control
**mvp_c2_traffic_control** is a node that manages the `/dccl_msg_tx` from either `reporter` or the `commander`. It manages the DCCL data coming from and flow into `reporter` and `commander`. More importantly, it controls when and which DCCL message will be send to the communication hardware.

### Published topics

### Subscribed topics

### Services

### Clients

### Parameters

# Full-duplex setup
TDMA to false

# Half-duplex setup (TDMA)
TDMA setup

# Multi-modal comm. setup

Multi-modal comm with the same TDMA config
Multi-modal comm with different TDMA config.
