
# mvp_c2_serial_comm

## Description
`mvp_c2_serial_comm` is a simple driver to interface with transparent serial communicaiton devices such as RFdesign 900ux and Xbee 900mHz.

## Published topics

| Topic | Type | Description |
|------|------|-------------|
| `/dccl_msg_rx` | `std_msgs/ByteMultiArray` | Messages received by the serial device |
---

## Subscribed topics

| Topic | Type | Description |
|------|------|-------------|
| `/dccl_msg_tx` | `std_msgs/ByteMultiArray` | Messages to be send out by the serial device|

---

## Parameters

- `port` (Default: `/dev/ttyUSB0`): 
Serial device path used for communication.  
  
- `baudrate` (Default: `9600`): 
  Serial communication baud rate.  
  
---
