# MVP C2 Traffic Control ROS node

## Description


## Published Topics

| Topic | Type | Description |
|------|------|-------------|
| `mvp_c2/traffic_control/dccl_msg_controlled_rx` | `std_msgs/ByteMultiArray` | To reporter or commander|
| `mvp_c2/traffic_control/dccl_msg_controlled_tx` | `std_msgs/ByteMultiArray` | To hardware|
| `mvp_c2/traffic_control/dccl_msg_controlled_tx_names` | `std_msgs/String` | DCCL message names sent to hardware|

---

## Subscribed Topics

| Topic | Type | Description |
|------|------|-------------|
| `mvp_c2/traffic_control/dccl_msg_tx` | `std_msgs/ByteMultiArray` | From reporter or commander|
| `mvp_c2/traffic_control/dccl_msg_rx` | `std_msgs/ByteMultiArray` | From hardware|
---

## Parameters
### general 
- `max_frame_size` (Default: `0.0`): The package size for each transmission

- `tx_interval` (Default: `1.0`): How often (seconds) to wait for fill up the packet.


### Dynamic buffer and message rules
- `dynamic_buffer.max_total_size` (Default: `0.0`): The maximum entries in dynamic buffer

- `dynamic_buffer.overflow_remove_by_time` (Default: `False`): remove the oldest entry in dynamic buffere regardless of their priority.

- `dynamic_buffer.dccl_intake_message_list` (Default: `[]`): a list of DCCL messages that will be stored in the dynamic buffer.
    ```
    # example 
    dccl_intake_message_list: 
            - "Odometry"
            - "GeoPose"
            - "PowerMonitor"
            - "CPUMonitor"
            - "AltimeterPointStamped"
            - "ReportPowerPort"
            - "ReportController"
            - "ReportHelm"
            - "ReportWpt"
            - "ReportRosLaunch"

    ```
- Define the `message rules`. Higher priority will be trainsmitted out first. `ttl_seconds` is how long the message will expire.
    ```
    #Example:
    message_rules:
      ReportController:
        priority: 10
        ttl_seconds: 100

      ReportHelm:
        priority: 9
        ttl_seconds: 100

      ReportWpt:
        priority: 8
        ttl_seconds: 100

      ReportRosLaunch:
        priority: 7
        ttl_seconds: 100

      ReportPowerPort:
        priority: 6
        ttl_seconds: 100

      AltimeterPointStamped:
        priority: 5
        ttl_seconds: 100

      CPUMonitor:
        priority: 4
        ttl_seconds: 100

      PowerMonitor:
        priority: 3
        ttl_seconds: 100

      GeoPose:
        priority: 2
        ttl_seconds: 100

      Odometry:
        priority: 1
        ttl_seconds: 5
    ```

### TDMA 
Our TDMA mechansim is designed such that the master will broadcast TDMA information after a defined numebr of TDMA cycles.
In sync slot, the master will broadcast TDMA settings, and slave will parse it and configure/adapt its TDMA setting.
The design is motivated such that master could adjust TDMA setting on-the-fly.
Below is an example that the Sync slot will occure after two  TDMA cycles

```
|<---------Full TDMA period sync------------>|

|           |<----TDMA Cycle----->|<----TDMA Cycle--->|

| Sync slot | slot1 | slot2| slot3| slot1|slot2| slot3|
```


- `tdma_enable` (Default: `False`): Enable or disable TDMA

- `tdma.slot_id` (Default: `0`): TDMA slot id

- `tdma.role`(Options: `master` or `slave`): TDMA roles. Master controls slot duration, number of slots and how to sync with others. `Salve` will automatically configure its TDMA based on master's sync message (DCCL message) broadcasted in the `time sync slot`

- `tdma.slot_duration`: what is the time duration for each slot.

- `tdma.num_slot`: total numbers of slots.

- `tdma.slot_guard_time_ms`: the blank time period at the beginning and end of each slot.

- `tdma.sync_slot_interval`: sync slot will occure after `<tdma.sync_slot_interval>` number of TDMA cycle.

- `tdma.sync_msg_repeat_num`: how many times the tdma sync message will be send out during the sync slot.


