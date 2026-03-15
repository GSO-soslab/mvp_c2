# MVP C2 UDP ROS Node

## Description
`mvp_c2_udp_comm` is a simple driver to send/receive data with transparent udp communicaiton devices such as Ubiquiti bullet.

## Published Topics

| Topic | Type | Description |
|------|------|-------------|
| `/dccl_msg_rx` | `std_msgs/ByteMultiArray` | Publishes messages received from the UDP socket. |

---

## Subscribed Topics

| Topic | Type | Description |
|------|------|-------------|
| `/dccl_msg_tx` | `std_msgs/ByteMultiArray` | Messages to be send out by the UDP device |

---

## Parameters

- `type` (Default: `server`) Defines the UDP mode.  
    Options: `server` for commander/topside; `client` for reporter/vehicle

- `server_ip` (Default: `192.168.0.123`): IP address of the `server`.  

- `server_port`( Default: `2000`) Port used by the `server`, less than `1023` may have access restrictions.  
 
- `client_ip` (Default: `192.168.0.100`): IP address of the `client`.  
  

- `client_port` (Default: `3000`): Port used by the `client`, less than `1023` may have access restrictions.   


