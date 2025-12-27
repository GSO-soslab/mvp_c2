import os
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, String
from ament_index_python.packages import get_package_share_directory
import dccl
import time
import mvp_cmd_dccl_pb2
from ament_index_python.packages import get_package_share_directory
from include.dynamic_buffer import DynamicBufferPython
from include.dccl_checksum import package_dccl, check_dccl
from google.protobuf import descriptor_pool


class TrafficControlRos(Node):
    def __init__(self):
        # IMPORTANT: Enable parameter overrides to read the YAML structure automatically
        super().__init__(
            'mvp_c2_traffic_control',
            automatically_declare_parameters_from_overrides=True,
            allow_undeclared_parameters=True
        )

        #load protobuff message
        proto_path = os.path.join(get_package_share_directory('mvp_c2'), 'proto', 'mvp_cmd_dccl.proto')

        if not os.path.exists(proto_path):
            self.get_logger().fatal(f"Proto file NOT found at: {proto_path}")
            raise FileNotFoundError(f"Could not find {proto_path}")

        dccl.loadProtoFile(proto_path)
        self.dccl_codec = dccl.Codec()

        #load dynamic buffer
        self.load_dynamic_buffer_config()
        
        #ros stuff
        self.dccl_tx_sub = self.create_subscription(ByteMultiArray, 'mvp_c2/traffic_control/dccl_msg_tx', self.dccl_tx_callback, 1) #from reporter/commander
        self.dccl_rx_sub = self.create_subscription(ByteMultiArray, 'mvp_c2/traffic_control/dccl_msg_rx', self.dccl_rx_callback, 1) #from hardware

        self.dccl_tx_pub = self.create_publisher(ByteMultiArray, 'mvp_c2/traffic_control/dccl_msg_controlled_tx', 10)  #to hardware
        self.dccl_tx_msg_pub = self.create_publisher(String, 'mvp_c2/traffic_control/dccl_msg_controlled_tx_names', 10)  #to hardware (dccl message name string array)

        self.dccl_rx_pub = self.create_publisher(ByteMultiArray, 'mvp_c2/traffic_control/dccl_msg_controlled_rx', 10)  #to reporter/commander

        # self.tx_interval = self.get_parameter('tx_interval').value
        # self.tx_msg_construction_timeout = self.get_parameter('tx_msg_construction_timeout').value

        self.tx_interval = self.get_parameter_or(
            'tx_interval', 1.0
        ).value


        self.tdma_enable = self.get_parameter_or('tdma_enable', False).value

        if self.tdma_enable:
            self.dccl_codec.load('TdmaMasterSyncMsg')
            self.load_tdma_config()

        self.start_time = None
        self.can_transmit_flag = True
        self.create_timer(0.01, self.dccl_pop_data) #pop data fequency
        self.create_timer(self.tx_interval, self.reset_transmit_flag)

    def reset_transmit_flag(self):
        #if still able transmit meaning no frame was transmitted, i will then transmit it
        if self.can_transmit_flag:
            self.push_frame()
        self.can_transmit_flag = True

    def load_tdma_config(self):
        #tdma starts in sync slot.
        #the master will broadcast messages.
        #Future the slave will response to the message in sequence, each one has time interval slot_duration/num_slots?
        # so we can adapt the slot 
        
        self.tdma_slot_id = self.get_parameter('tdma.slot_id').value
        self.tdma_role = self.get_parameter('tdma.role').value
        
        if self.tdma_role == "master":
            self.tdma_slot_duration = self.get_parameter_or('tdma.slot_duration', 1.0).value
            self.tdma_num_slots = self.get_parameter_or('tdma.num_slots', 1 ).value  
            self.tdma_slot_guard_time_ms = self.get_parameter_or('tdma.slot_guard_time_ms', 0).value  
            #the sync message will be set after n frames
            self.tdma_sync_slot_interval = self.get_parameter_or('tdma.sync_slot_interval', -1).value
            #how many sync message will be set? timed by slot_duration/tdma_sync_msg_repeat_num
            self.tdma_sync_msg_repeat_num = self.get_parameter_or('tdma.sync_msg_repeat_num', 0).value   
            #if sync happens the tdma time can be reset 
            self.master_sync_slot()

        else:
            self.tdma_slot_duration = 0
            self.tdma_num_slots = 0 
            self.tdma_slot_guard_time_ms = 0
            #the sync message will be set after n frames
            self.tdma_sync_slot_interval = 0
            #how many sync message will be set? timed by slot_duration/tdma_sync_msg_repeat_num
            self.tdma_sync_msg_repeat_num = 0  
        
        #set tdma_setup_flag to True for sync msg and handle.
        self.tdma_flag = False
        self.tdma_in_slot = True

    def master_sync_slot(self):

        self.tdma_flag = False
        #reset tdma start_time
        self.tdma_start_time = round(time.time(), 1) 
        
        #prepare the msg except the time
        proto = mvp_cmd_dccl_pb2.TdmaMasterSyncMsg()
        proto.time = round(time.time(), 1)
        proto.start_time = self.tdma_start_time
        proto.sync_slot_interval = self.tdma_sync_slot_interval
        proto.slot_duration = self.tdma_slot_duration
        proto.slot_guard_time_ms = self.tdma_slot_guard_time_ms
        proto.num_slots = self.tdma_num_slots
        proto.slot_id = self.tdma_slot_id

        print("proto data")
        
        print(proto, flush=True)
        dccl_msg = self.dccl_codec.encode(proto)

        print("####", flush=True)
        #compute the delay between messages
        #|--------------slot---------------|
        #|guard_time|msg|msg|msg|guard_time|
        sync_msg_interval = (self.tdma_slot_duration-2*self.tdma_slot_guard_time_ms/1000)/self.tdma_sync_msg_repeat_num
        sync_msg_start_t  = self.tdma_start_time + self.tdma_slot_guard_time_ms/1000 
        #wait for the guard time
        while time.time() < sync_msg_start_t:
            # print("Waiting the guardtime", flush=True)
            time.sleep(0.001)
        
        for i in range(self.tdma_sync_msg_repeat_num):
            proto.time = round(time.time(), 1)
            msg_send_time = sync_msg_start_t + sync_msg_interval*(i+1)
            while time.time() <  msg_send_time:
                time.sleep(0.001)
            #send the message
            dccl_msg = self.dccl_codec.encode(proto)
            dccl_msg = package_dccl(dccl_msg)
            self.output_buffer.extend(dccl_msg)
            self.output_msg_names += f", {'TdmaMasterSyncMsg'}" 
            print("Sending Sync Msgs", flush=True)
            self.push_frame()    
        
        self.tdma_flag = True

    def tdma_slave_update(self,data):
        #decode the data
        self.tdma_flag = False
        proto_msg = self.dccl_codec.decode(data)
        host_time = proto_msg.time #use for time sync

        self.tdma_start_time = proto_msg.start_time
        self.tdma_sync_slot_interval = proto_msg.sync_slot_interval
        self.tdma_slot_duration = proto_msg.slot_duration
        self.tdma_slot_guard_time_ms = proto_msg.slot_guard_time_ms
        self.tdma_num_slots = proto_msg.num_slots

        self.tdma_flag = True

    def tdma_in_slot_check(self):
       #setup tdma

        #|--------|---------Frame-----|---------Frame-----|
        #sync_slot|slot|slot|slot|slot|slot|slot|slot|slot|
        #sync_slot_index = k(n_slots*tdma_sync_slot_interval + 1)
        #start from 0
        #k(n_slots*tdma_sync_slot_interval + 1)
        if not self.tdma_flag:
            self.get_logger().warn(
                "TDMA_flag is not setup",
                throttle_duration_sec=1.0
            )
            return False
        
        current_time = round(time.time(), 1) 
        tdma_elaspsed_time = current_time - self.tdma_start_time

        slot_count = int(tdma_elaspsed_time //self.tdma_slot_duration)
        cycle_slots = self.tdma_num_slots * self.tdma_sync_slot_interval + 1
        cycle_count = slot_count % cycle_slots
        
        if cycle_count == 0:
            if self.tdma_role == "master":
                self.master_sync_slot()
            return False
        else: 
            in_slot_node = (cycle_count - 1) % self.tdma_num_slots
            #check slot ID
            if in_slot_node != self.tdma_slot_id:
                self.get_logger().warn(
                    f"Not my slot: current_slot={in_slot_node}, my_slot={self.tdma_slot_id}",
                    throttle_duration_sec=1.0
                )
                return False
            
            slot_elapsed = tdma_elaspsed_time % self.tdma_slot_duration
            #check guard time
            allowed_start_time = self.tdma_slot_guard_time_ms/1000
            allowed_end_time = self.tdma_slot_duration-self.tdma_slot_guard_time_ms/1000
            if allowed_start_time < slot_elapsed < allowed_end_time:
                return True
            else:
                print("In guard time", flush=True)
                self.get_logger().warn(
                "In guard time",
                throttle_duration_sec=1.0
            )
                return False

    def load_dynamic_buffer_config(self):

        max_size = self.get_parameter('dynamic_buffer.max_total_size').value

        buffer_overflow_remove_by_time = self.get_parameter('dynamic_buffer.overflow_remove_by_time').value

        self.max_frame_size = self.get_parameter('max_frame_size').value

        # BUffering parameter
        self.output_buffer = bytearray()
        self.output_msg_names = "" # This will track the names
        # self.max_frame_size = self.declare_parameter('max_frame_size', 128).value 

        # self.allowed_messages = self.get_parameter('dynamic_buffer.dccl_intake_message_list').value
        msg_list_param = self.get_parameter('dynamic_buffer.dccl_intake_message_list')

        #Safely skip the messages if there is no conifguration
        self.allowed_messages = msg_list_param.value if msg_list_param.value is not None else []
        if not self.allowed_messages:
            self.get_logger().warn("No messages found in 'dccl_intake_message_list'. Buffer will be idle.")
            self.message_rules = {}
            self.dynamic_buffer = DynamicBufferPython(max_total_size=max_size, drop_by_time = buffer_overflow_remove_by_time)
            return

        self.message_rules = {}
        print("loading dccl message for dynamic buffer")
        for msg_name in self.allowed_messages:
            print(msg_name, flush = True)
            self.dccl_codec.load(msg_name) #load proto data
            self.message_rules[msg_name] = {
                'priority': self.get_parameter(f'message_rules.{msg_name}.priority').value,
                'ttl_seconds':      self.get_parameter(f'message_rules.{msg_name}.ttl_seconds').value,
                'group':    msg_name
            }

        self.dynamic_buffer = DynamicBufferPython(max_total_size=max_size)
    
    def dccl_rx_callback(self, msg):
        # print("Parsing msg into multiple dccl msgs")
        data = bytearray(ord(c) for c in msg.data) 
        
        dccl_msg = bytearray()

        for i in range(len(data)):
            dccl_msg.append(data[i])

            if len(dccl_msg) >= 4 and dccl_msg[-4] == 42 and dccl_msg[-1]==ord('\n'): #the four last chars are *AB\n
                #check and peak the message
                flag, cdata = check_dccl(dccl_msg)
                message_id = self.dccl_codec.id(cdata)
                #if it is master sync message i will update the tdma setting
                if flag and message_id == 51:
                    self.tdma_slave_update(cdata)
                #message will still be published so we can bag
                msg = ByteMultiArray()
                msg.data = dccl_msg
                self.dccl_rx_pub.publish(msg)
                dccl_msg = bytearray()
                 
    def dccl_tx_callback(self, msg):
        try:
            raw_bytes = bytearray(ord(c) for c in msg.data)
            raw_bytes = bytes(raw_bytes)
            # raw_bytes = bytes(msg.data)
            # Extract DCCL core (Strips $$$ and *XX\n)
            dccl_payload = raw_bytes[2:-4]
            
            # Peek at type
            decoded_obj = self.dccl_codec.decode(dccl_payload)
            msg_name = decoded_obj.DESCRIPTOR.name 
            # self.get_logger().info(f"Processing DCCL message: {msg_name}")

            if msg_name in self.message_rules:
                rules = self.message_rules[msg_name]
                # Push the full NMEA-framed packet to the buffer
                self.dynamic_buffer.push(
                    data=raw_bytes, 
                    priority=rules['priority'], 
                    ttl_seconds=rules['ttl_seconds'], 
                    group_name=rules['group']
                )
        except Exception as e:
            self.get_logger().error(f"Failed to process DCCL intake: {e}")

    def dccl_pop_data(self):

        if self.tdma_enable:
            if not self.tdma_in_slot_check():
                # print("Not in my slot")
                return

        #not ready i will skip
        if not self.can_transmit_flag:
            return
        
        new_data, msg_name = self.dynamic_buffer.pop()

        if not new_data:
            return
        
        #if new data will saturate by buffer
        if self.output_buffer and (len(self.output_buffer) + len(new_data) > self.max_frame_size):
            if msg_name:
                if self.output_msg_names == "":
                    self.output_msg_names = msg_name
                else:
                    self.output_msg_names += f", {msg_name}" # Add separator
            self.push_frame()
            # log the data for the next time
            self.output_buffer.extend(new_data)
            self.start_time = None
            return
        
        #regular loop accumulating data
        self.output_buffer.extend(new_data)
        if msg_name:
            if self.output_msg_names == "":
                self.output_msg_names = msg_name
            else:
                self.output_msg_names += f", {msg_name}" # Add separator
    
        #if one dccl already exceed the max frame size
        if len(self.output_buffer) >= self.max_frame_size:
            self.push_frame()
            self.start_time = None

    def push_frame(self):
        out_msg = ByteMultiArray()
        out_msg.data = bytearray(self.output_buffer)
        self.dccl_tx_pub.publish(out_msg)
        # self.get_logger().info(f"Buffer data length: {len(out_msg.data)}")
        name_msg = String()
        name_msg.data = self.output_msg_names
        self.dccl_tx_msg_pub.publish(name_msg)
        #reset the buffer 
        self.output_buffer = bytearray()
        self.output_msg_names = ""
        self.can_transmit_flag = False #rest the flag to false and wait for it to become true

def main(args=None):
    rclpy.init(args=args)
    node = TrafficControlRos()
    rclpy.spin(node)  # Keep the node running to allow timer execution
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()