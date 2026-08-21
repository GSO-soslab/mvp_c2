import os
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, String
from std_srvs.srv import Trigger

from mvp_msgs.srv import SetString
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

        self.dccl_codec.load('TdmaMasterSyncMsg')

        

        
        #ros stuff
        self.dccl_tx_sub = self.create_subscription(ByteMultiArray, 'mvp_c2/traffic_control/dccl_msg_tx', self.dccl_tx_callback, 10) #from reporter/commander
        self.dccl_tx_msg_pub = self.create_publisher(String, 'mvp_c2/traffic_control/dccl_msg_controlled_tx_names', 10)  #dccl message name string array
        self.dccl_rx_pub = self.create_publisher(ByteMultiArray, 'mvp_c2/traffic_control/dccl_msg_controlled_rx', 10)  #to reporter/commander

        self.set_comm_service = self.create_service(
            SetString,
            'mvp_c2/traffic_control/set_current_comm',
            self.set_comm_callback
        )

        self.get_current_comm_service = self.create_service(
            Trigger,
            'mvp_c2/traffic_control/get_current_comm',
            self.get_current_comm_callback
        )

        self.get_all_comm_service = self.create_service(
            Trigger,
            'mvp_c2/traffic_control/get_all_comms',
            self.get_all_comm_callback
        )

        #load comm types and settings
        self.load_comm_conifg()
        print("setup complete")

        

        self.can_transmit_flag = True
        self.create_timer(1.0, self.dccl_pop_data) #pop data fequency
        self.last_push_time = time.time() ##my wait time for the push

    def load_comm_conifg(self):
        self.comm_list = self.get_parameter('comm_list').value
        self.auto_comm_switching = self.get_parameter_or('auto_comm_switching', False).value #true for reporter side
        self.comm_settings = {}
        self.tdma_settings = {}
        self.dynamic_buffer_settings = {}
        self.dynamic_buffer_message_rules = {}

        self.current_comm = self.comm_list[0]
        # loop through the comm list
        for comm_name in self.comm_list:

            #setup sub and pub
            self.self.dccl_tx_pubs[comm_name] = self.create_publisher(
                                                ByteMultiArray, 
                                                f'mvp_c2/traffic_control/{comm_name}/dccl_msg_controlled_tx', 
                                                10)  #to hardware
            
            self.dccl_rx_sub = self.create_subscription(
                ByteMultiArray,
                f'mvp_c2/traffic_control/{comm_name}/dccl_msg_rx',
                lambda msg, comm_name=comm_name: self.dccl_rx_callback(msg, comm_name),
                10
            )  #from hardware
            
            #load basic settings
            self.comm_settings[comm_name] = {
                'max_frame_size': self.get_parameter(f'{comm_name}.max_frame_size').value,
                'tx_interval':  self.get_parameter_or(f'{comm_name}.tx_interval').value,
            }

            #load dynamic buffer setting
            self.dynamic_buffer_settings[comm_name] ={
                'max_total_size': self.get_parameter(f'{comm_name}.dynamic_buffer.max_total_size').value,
                'buffer_overflow_remove_by_time': self.get_parameter(f'{comm_name}.dynamic_buffer.overflow_remove_by_time').value,
                'msgs_list': self.get_parameter(f'{comm_name}.dynamic_buffer.dccl_intake_message_list').value
            }
            
            #Safely skip the messages if there is no conifguration
            allowed_messages = (self.dynamic_buffer_settings[comm_name]['msgs_list'] or [])
            # self.allowed_messages = self.dynamic_buffer_settings[comm_name]['msgs_list'].value if self.dynamic_buffer_settings[comm_name]['msgs_list'].value is not None else []
            if not allowed_messages:
                self.get_logger().warn("No messages found in 'dccl_intake_message_list'. Buffer will be idle.")
                self.dynamic_buffer_message_rules[comm_name] = {}
                continue

            print("loading dccl message for dynamic buffer")

            #load message settings
            self.dynamic_buffer_message_rules[comm_name] = {}

            for msg_name in allowed_messages:
                # print(msg_name, flush = True)
                self.dccl_codec.load(msg_name) #load proto data
                self.dynamic_buffer_message_rules[comm_name][msg_name] = {
                    'priority': self.get_parameter(f'{comm_name}.message_rules.{msg_name}.priority').value,
                    'ttl_seconds':      self.get_parameter(f'{comm_name}.message_rules.{msg_name}.ttl_seconds').value,
                    'group':    msg_name
                }

            #load tdma settings
            self.tdma_settings[comm_name] = {
                'enable': self.get_parameter_or(f'{comm_name}.tdma_enable',False).value,
                'slot_id': self.get_parameter(f'{comm_name}.tdma.slot_id').value,
                'role': self.get_parameter(f'{comm_name}.tdma.role').value
            }

            #tdma master
            if self.tdma_settings[comm_name]['role'] == "master":
                self.tdma_settings[comm_name].update({
                'slot_duration': self.get_parameter(f'{comm_name}.tdma.slot_duration').value,
                'num_slots':  self.get_parameter(f'{comm_name}.tdma.num_slots').value,
                'slot_guard_time_ms': self.get_parameter(f'{comm_name}.tdma.slot_guard_time_ms').value,
                #the sync message will be set after n frames
                #how many sync message will be set? timed by slot_duration/tdma_sync_msg_repeat_num
                'sync_slot_interval': self.get_parameter_or(f'{comm_name}.tdma.sync_slot_interval').value,
                'sync_msg_repeat_num': self.get_parameter_or(f'{comm_name}.tdma.sync_msg_repeat_num').value 

            })
                
            #tdma slave
            else:
                self.tdma_settings[comm_name].update({
                    'slot_duration': 0.0,
                    'num_slots':  0,
                    'slot_guard_time_ms': 0,
                    #the sync message will be set after n frames
                    #how many sync message will be set? timed by slot_duration/tdma_sync_msg_repeat_num
                    'sync_slot_interval': 0,
                    'sync_msg_repeat_num': 0 
    
                })

        self.reset_comm(self.comm_list[0])

            
    def get_current_comm_callback(self, request, response):
        response.success = True
        response.message = self.current_comm
        return response

    def get_all_comm_callback(self, request, response):
        response.success = True
        response.message = ','.join(self.comm_list)
        return response
    
    def set_comm_callback(self, request, response):
        #resetting the comm and buffer
        flag = self.reset_comm(request.data)
        if flag:
            response.success = True
            response.message = f'comm is set to {request.data}'
        else:
            response.success = True
            response.message = f'wrong comm name'
        return response
    
    def reset_comm(self, comm_name):
        if comm_name in self.comm_list:
            print(f"Found: {comm_name}")

            self.max_frame_size = self.comm_settings[comm_name]['max_frame_size']
            self.tx_interval = self.comm_settings[comm_name]['tx_interval']
            self.dynamic_buffer = DynamicBufferPython(max_total_size=self.dynamic_buffer_settings[comm_name]['max_total_size'],
                                                        drop_by_time = self.dynamic_buffer_settings[comm_name]['buffer_overflow_remove_by_time'])
            
            self.message_rules = self.dynamic_buffer_message_rules[comm_name]
            self.tdma_enable = self.tdma_settings[comm_name]['enable']
            self.tdma_slot_id = self.tdma_settings[comm_name]['slot_id']
            self.tdma_role = self.tdma_settings[comm_name]['role']

            if self.tdma_enable:
                self.tdma_flag = False
                if self.tdma_role == "master":
                    self.tdma_slot_duration = self.tdma_settings[comm_name]['slot_duration']
                    self.tdma_num_slots = self.tdma_settings[comm_name]['num_slots']
                    self.tdma_slot_guard_time_ms = self.tdma_settings[comm_name]['slot_guard_time_ms']
                    self.tdma_sync_slot_interval = self.tdma_settings[comm_name]['sync_slot_interval']
                    self.tdma_sync_msg_repeat_num = self.tdma_settings[comm_name]['sync_msg_repeat_num']
                    print("#####TDMA  enabled####", flush = True)
                    self.master_sync_slot()
                else:
                    self.tdma_slot_duration = 0.0
                    self.tdma_num_slots = 0
                    self.tdma_slot_guard_time_ms = 0
                    self.tdma_sync_slot_interval = 0
                    self.tdma_sync_msg_repeat_num = 0
                                
            self.last_push_time = time.time() ##my wait time for the push
             # initialize the output buffer
            self.output_buffer = bytearray()
            self.output_msg_names = ""
            self.current_comm = comm_name
            return True
        else:
            return False


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

        # print("proto data")
        
        print(proto, flush=True)
        # dccl_msg = self.dccl_codec.encode(proto)

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

        #wait until the end of this slot
        while time.time() < (self.tdma_start_time + self.tdma_slot_duration):
            # print("Waiting the guardtime", flush=True)
            time.sleep(0.001)
        self.tdma_flag = True
        print("TDMA_Sync done", flush=True)

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

        print("TDMA_Sync Msg from Master Received", flush=True)

        self.tdma_flag = True

    def tdma_in_slot_check(self):

        if not self.tdma_enable:
            return True
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

        # print(tdma_elaspsed_time, flush= True)

        slot_count = int(tdma_elaspsed_time //self.tdma_slot_duration)
        cycle_slots = self.tdma_num_slots * self.tdma_sync_slot_interval + 1
        cycle_count = slot_count % cycle_slots
        
        if cycle_count == 0:
            if self.tdma_role == "master":
                print("Sync again", flush=True)
                self.master_sync_slot()
            return False
        else: 
            in_slot_node = (cycle_count - 1) % self.tdma_num_slots
            #check slot ID
            if in_slot_node != self.tdma_slot_id:
                self.get_logger().warn(
                    f"Not my slot: Slot_status=[{cycle_count}/{cycle_slots}]| current/my=[{in_slot_node}/{self.tdma_slot_id}]",
                    throttle_duration_sec=1.0
                )
                return False
            
            slot_elapsed = tdma_elaspsed_time % self.tdma_slot_duration
            #check guard time
            allowed_start_time = self.tdma_slot_guard_time_ms/1000
            allowed_end_time = self.tdma_slot_duration-self.tdma_slot_guard_time_ms/1000
            if allowed_start_time < slot_elapsed < allowed_end_time:
                self.get_logger().info(
                    f"In my slot: my_slot/total=[{self.tdma_slot_id}/{self.tdma_num_slots}]",
                    throttle_duration_sec=1.0
                )
                return True
            else:
                self.get_logger().warn(
                "In guard time",
                throttle_duration_sec=1.0
            )
                return False

    def dccl_rx_callback(self, msg, comm_name):
        
        if comm_name != self.current_comm:
            #switch the current comm.
            if self.auto_comm_switching:
                self.current_comm = comm_name
            else:
                return
        
        # print("Parsing msg into multiple dccl msgs")
        data = bytearray(ord(c) for c in msg.data) 
        
        dccl_msg = bytearray()
        search_start = True

        for i in range(len(data)):
            #detect the start first.
            if search_start and data[i] == ord('$'):
                search_start = False
                dccl_msg = bytearray()

            if not search_start:
                dccl_msg.append(data[i])
                
                if len(dccl_msg) >= 7 and dccl_msg[-4] == 42 and dccl_msg[-1]==ord('\n'): #the four last chars are *AB\n
                    #check and peak the message
                    flag, cdata = check_dccl(dccl_msg)
                    if flag:
                        try:
                            message_id = self.dccl_codec.id(cdata)
                        except Exception as e:
                            print("Could not Deode!", flush=True)
                            return
                        except dccl.DcclException:
                            print("Could not Deode!", flush=True)
                            return

                    #if it is master sync message i will update the tdma setting
                    if flag and message_id == 51:
                        self.tdma_slave_update(cdata)
                        #not sending down
                        # return
                    #message will still be published so we can bag
                    msg = ByteMultiArray()
                    msg.data = dccl_msg
                    self.dccl_rx_pub.publish(msg)
                    dccl_msg = bytearray()
                    search_start = True
                 
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
            else:
                self.get_logger().info(f"{msg_name} is descarded because is not included in the intake param")
                
        except Exception as e:
            self.get_logger().error(f"Failed to process DCCL intake: {e}")

    def dccl_pop_data(self):
        print("looping")
        if self.tdma_enable:
            if not self.tdma_in_slot_check():
                # print("Not in my slot")
                return
                
        if time.time()-self.last_push_time > self.tx_interval:
            self.push_frame()
            # print("Push wait time has reached", flush = True)

        #not ready i will skip
        # if not self.can_transmit_flag:
            # return
        
        new_data, msg_name = self.dynamic_buffer.pop()

        if not new_data:
            # self.get_logger().warn(
            #         f"No data popped: [{self.dynamic_buffer._queue}]",
            #         throttle_duration_sec=1.0
            # )
            return
        
        if msg_name in self.output_msg_names:
            # self.get_logger().warn(
            #         f"Skip the same message: [{msg_name}]",
            #         throttle_duration_sec=1.0
            # )
            return

        #if new data will saturate by buffer
        if self.output_buffer and (len(self.output_buffer) + len(new_data) > self.max_frame_size):
            self.push_frame()
            if msg_name:
                if self.output_msg_names == "":
                    self.output_msg_names = msg_name
                else:
                    self.output_msg_names += f", {msg_name}" # Add separator
            
            # log the data for the next time
            self.output_buffer.extend(new_data)
            # self.start_time = None
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
            # self.start_time = None

    def push_frame(self):
        out_msg = ByteMultiArray()
        self.last_push_time = time.time()
        if self.output_buffer:
            out_msg.data = bytearray(self.output_buffer)
            self.dccl_tx_pub[self.current_comm].publish(out_msg)
            # self.get_logger().info(f"Buffer data length: {len(out_msg.data)}")
            name_msg = String()
            name_msg.data = self.output_msg_names
            self.dccl_tx_msg_pub.publish(name_msg)
            #reset the buffer 
            self.output_buffer = bytearray()
            self.output_msg_names = ""
            print("### Total entries in Dynamic buffer:", len(self.dynamic_buffer._queue), flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficControlRos()
    rclpy.spin(node)  # Keep the node running to allow timer execution
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()