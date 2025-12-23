import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, String
from ament_index_python.packages import get_package_share_directory
import dccl
import mvp_cmd_dccl_pb2
from ament_index_python.packages import get_package_share_directory
from include.dynamic_buffer import DynamicBufferPython
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

        self.create_timer(0.01, self.dccl_pop_data) #pop data fequency
    
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
        print("Parsing msg into multiple dccl msgs")
        data = bytearray(ord(c) for c in msg.data) 

        dccl_msg = bytearray()

        for i in range(len(data)):
            dccl_msg = dccl_msg.append(data[i])

            if len(dccl_msg) >= 4 and dccl_msg[-4] == 42: #the four last chars are *AB\n
                msg = ByteMultiArray()
                msg.data = dccl_msg
                # print(msg.data)
                # print(f'received:{len(msg.data)}', flush=True)
                self.ddcl_rx_pub.publish(msg)
                print("publishing", flush = True)
                
                dccl_msg = bytearray()

        # self.dccl_rx_pub.publihs(msg)
        # print("check which hardware was")
        print("Forward to reporter/commander", flush=True)

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
        new_data, msg_name = self.dynamic_buffer.pop()

        if not new_data:
            return
        
        if self.output_buffer and (len(self.output_buffer) + len(new_data) > self.max_frame_size):
            self.push_frame()

        # accumulate data
        self.output_buffer.extend(new_data)
        if msg_name:
            if self.output_msg_names == "":
                self.output_msg_names = msg_name
            else:
                self.output_msg_names += f", {msg_name}" # Add separator

        #if one dccl already exceed the max frame size
        if len(self.output_buffer) >= self.max_frame_size:
            self.push_frame()

    def push_frame(self):
        out_msg = ByteMultiArray()
        out_msg.data = bytearray(self.output_buffer)
        self.dccl_tx_pub.publish(out_msg)
        self.get_logger().info(f"Buffer data length: {len(out_msg.data)}")

        name_msg = String()
        name_msg.data = self.output_msg_names
        self.dccl_tx_msg_pub.publish(name_msg)
        #reset the buffer 
        self.output_buffer = bytearray()
        self.output_msg_names = ""

def main(args=None):
    rclpy.init(args=args)
    node = TrafficControlRos()
    rclpy.spin(node)  # Keep the node running to allow timer execution
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()