import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
from ament_index_python.packages import get_package_share_directory
# import dccl
# from your_module import DynamicBufferPython

class TrafficControlRos(Node):
    def __init__(self):
        # IMPORTANT: Enable parameter overrides to read the YAML structure automatically
        super().__init__(
            'mvp_c2_traffic_control',
            automatically_declare_parameters_from_overrides=True,
            allow_undeclared_parameters=True
        )

        # 1. Load Global Settings
        max_size = self.get_parameter('dynamic_buffer.max_total_size').value
        self.allowed_messages = self.get_parameter('dynamic_buffer.dccl_intake_message_list').value
        
        self.dynamic_buffer = DynamicBufferPython(max_total_size=max_size)

        # 2. Setup DCCL
        proto_path = os.path.join(get_package_share_directory('mvp_c2'), 'proto', 'mvp_cmd_dccl.proto')
        # dccl.loadProtoFile(proto_path)
        # self.dccl_codec = dccl.Codec()

        # 3. ROS Interfaces
        self.dccl_intake_sub = self.create_subscription(
            ByteMultiArray, 
            'mvp_c2/dccl_msg_tx', 
            self.dccl_tx_callback, 
            10)
            
        self.get_logger().info("Traffic Control Node started with YAML rules.")

    def dccl_tx_callback(self, msg):
        try:
            # Decode DCCL message
            raw_bytes = bytes(msg.data)
            decoded_obj = self.dccl_codec.decode(raw_bytes)
            
            # Get the message name (e.g., "Odometry")
            # Note: Depending on your DCCL version, this might be decoded_obj.DESCRIPTOR.name
            msg_name = decoded_obj.DESCRIPTOR.name 

            # Filter messages not in our intake list
            if msg_name not in self.allowed_messages:
                return

            # --- DYNAMIC LOOKUP FROM YAML ---
            # We construct the parameter path string dynamically
            priority = self.get_parameter(f'message_rules.{msg_name}.priority').value
            ttl      = self.get_parameter(f'message_rules.{msg_name}.ttl').value
            group    = self.get_parameter(f'message_rules.{msg_name}.group').value

            # Push to the dynamic buffer
            self.dynamic_buffer.push(decoded_obj, priority, ttl, group_name=group)
            
            self.get_logger().debug(f"Buffered {msg_name}: P={priority}, G={group}")

        except Exception as e:
            self.get_logger().error(f"Failed to process DCCL intake: {e}")