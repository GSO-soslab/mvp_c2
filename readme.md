# Introduction
**mvp_c2** is a python-based ROS2 package designed for marine vehicle control and command via multiple ways of communications, such as RF, WIFI, and acoustic modem.

**Data encoding/decoding** is based on `DCCL` library. Each encoded DCCL message is argumented with header `$$`, `XOR` checksum, and `\n` ending.

For the vehicle with transparent RF or WIFI communication, we have serial and udp ROS nodes that could be used.
For acoustic modems, we normally need a customized sensor drivers which could package data in specific format that is compatiable with acoustic modem software protocol. One example can be found [here](https://github.com/GSO-soslab/evologics_ros/tree/jazzy-devel) for evologics USBL/Acoustic modem device.

Two way communication are implemented using a priority based dynamic buffer mechanism and optional TDMA scheduling mechanism with programmable time synch occurence slot.

# ROS nodes
## mvp_c2_serial_comm 
`mvp_c2_serial_comm` is a simple driver to interface with transparent serial communicaiton devices such as RFdesign 900ux and Xbee 900mHz. [Further document](mvp_c2_serial_comm_node.md)

## mvp_c2_udp_comm 
`mvp_c2_udp_comm` is a simple driver to send/receive data with transparent udp communicaiton devices such as Ubiquiti bullet. [Further document](mvp_c2_udp_comm_node.md)


## mvp_c2_reporter
`mvp_c2_report` is running on the vehicle side. It reports vehicle status, including odometry, geoposition, roslaunch file, power port, waypoints, helm state, and controller state, and etc.. The information are encoded into DCCL message and published to `mvp_c2/reporter/dccl_msg_tx` topic.
Meanwhile, it subscribeds to `mvp_c2/reporter/dccl_msg_rx` topic. It parse all the messages coming from this topic and publish to other topics or call services created by vehicle controller and helm software.
We don't configure this node directly interface with the communication hardware drivers. Instead, we add a `traffic controller` between them to manage the DCCL data traffic.
[Further document](mvp_c2_reporter.md)


## mvp_c2_commander
**mvp_c2_commander** is running on the topside or a master robot. It creates ros services and topics allowing users to start ROS launch files, change helm state, switch controller state, and update waypoints of a remote vehicle. The commands are encoded into DCCL message and published to `mvp_c2/commander/dccl_msg_tx` topic as a `ByteMultiArray` ROS message. Meanwhile, the commander subscribes to `mvp_c2_commander/dccl_msg_rx` topic to receive DCCL messages, then decode them and publish to different topics.
[Further document](mvp_c2_commander.md)

## mvp_c2_traffic_control
**mvp_c2_traffic_control** is a node that manages the `/dccl_msg_tx` from either `reporter` or the `commander`. It manages the DCCL data coming from and flow into `reporter` and `commander`. More importantly, it controls when and which DCCL message will be send to the communication hardware.


# Setup instruction
## Simple data flow (One to one)
### Commander to reporter
- **Commander:** Topic/service called by user → |package DCCL| → `mvp_c2/commander/dccl_msg_tx`→

- **Commander traffic control:** → `mvp_c2/traffic_control/dccl_msg_tx` → |dynamic buffer| → |TDMA (optional)| →`mvp_c2/traffic_control/dccl_msg_controlled_tx`→

- **Hardware communication:**  → `dccl_msg_tx` (hardware_tx_topic)

    -----------Wireless--------------

- **Hardware communication:** → `dccl_msg_rx` (hardware_rx_topic)→

- **Reporter traffic control:** →`mvp_c2/traffic_control/dccl_msg_rx`→|salinity check| →`mvp_c2/traffic_control/dccl_msg_controlled_rx`→

- **Reporter:** →`mvp_c2/reporter/dccl_msg_rx` → |salinity check| → |remote_id check| → |time interval check| → parse→action


### Reporter to commander
- **Reporter:**  Topic or client call → |package DCCL| → `mvp_c2/reporter/dccl_msg_tx`→
- Reporter traffic control: → `mvp_c2/traffic_control/dccl_msg_tx`→ |dynamic buffer| → |TDMA (optional)| →`mvp_c2/traffic_control/dccl_msg_controlled_tx`→

- **Hardware communication:** → `dccl_msg_tx` (hardware_tx_topic)

     -----------Wireless--------------

- **Hardware communication:** → `dccl_msg_rx` (hardware_rx_topic)→

- **Commander traffic control:** →`mvp_c2/traffic_control/dccl_msg_rx` → |salinity check| →`mvp_c2/traffic_control/dccl_msg_controlled_rx`→

- **Commander:** → `mvp_c2/commander/dccl_msg_rx` → |salinity check| → |remote_id check| → |time interval check| → parse→action/publish


## Full-duplex setup
TDMA to false

## Half-duplex setup (TDMA)
TDMA setup

## Multi-modal comm. setup

Multi-modal comm with the same TDMA config
Multi-modal comm with different TDMA config.
