import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
from ament_index_python.packages import get_package_share_directory
import dccl
import mvp_cmd_dccl_pb2
from ament_index_python.packages import get_package_share_directory
from include.dynamic_buffer import DynamicBufferPython


class TrafficControlRos(Node):
    def __init__(self):
        # IMPORTANT: Enable parameter overrides to read the YAML structure automatically
        super().__init__(
            'mvp_c2_traffic_control',
            automatically_declare_parameters_from_overrides=True,
            allow_undeclared_parameters=True
        )

        self.load_dynamic_buffer_config()
       
        #load protobuff message
        proto_path = os.path.join(get_package_share_directory('mvp_c2'), 'proto', 'mvp_cmd_dccl.proto')
        dccl.loadProtoFile(proto_path)
        self.dccl_codec = dccl.Codec()

        #ros stuff
        self.dccl_tx_sub = self.create_subscription(ByteMultiArray, 'mvp_c2/traffic_manager/dccl_msg_tx', self.dccl_tx_callback, 10) #from reporter/commander
        self.dccl_rx_sub = self.create_subscription(ByteMultiArray, 'mvp_c2/traffic_manager/dccl_msg_rx', self.dccl_rx_callback, 10) #from hardware

        self.dccl_tx_pub = self.create_publisher(ByteMultiArray, 'mvp_c2/traffic_manager/dccl_msg_control_tx', 10)  #to hardware
        self.dccl_rx_pub = self.create_publisher(ByteMultiArray, 'mvp_c2/traffic_manager/dccl_msg_control_rx', 10)  #to reporter/commander

        self.create_timer(0.1, self.dccl_pop_data)
    
    def load_dynamic_buffer_config(self):
        max_size = self.get_parameter('dynamic_buffer.max_total_size').value
        self.allowed_messages = self.get_parameter('dynamic_buffer.dccl_intake_message_list').value

        self.message_rules = {}
        for msg_name in self.allowed_messages:
            self.message_rules[msg_name] = {
                'priority': self.get_parameter(f'message_rules.{msg_name}.priority').value,
                'ttl':      self.get_parameter(f'message_rules.{msg_name}.ttl').value,
                'group':    self.get_parameter(f'message_rules.{msg_name}.group').value
            }

        self.dynamic_buffer = DynamicBufferPython(max_total_size=max_size)
    
    def dccl_rx_callback(self, msg):
        print("check which hardware was")
        print("Forward to reporter/commander")

    def dccl_tx_callback(self, msg):
        try:
            raw_bytes = bytes(msg.data)
            
            # Extract DCCL core (Strips $$$ and *XX\n)
            dccl_payload = raw_bytes[3:-4]
            
            # Peek at type
            decoded_obj = self.dccl_codec.decode(dccl_payload)
            msg_name = decoded_obj.DESCRIPTOR.name 

            if msg_name in self.message_rules:
                rules = self.message_rules[msg_name]
                # Push the full NMEA-framed packet to the buffer
                self.dynamic_buffer.push(
                    data=raw_bytes, 
                    priority=rules['priority'], 
                    ttl=rules['ttl'], 
                    group_name=rules['group']
                )
        except Exception as e:
            self.get_logger().error(f"Failed to buffer local message: {e}")

    
        except Exception as e:
            self.get_logger().error(f"Failed to process DCCL intake: {e}")


        def dccl_pop_data(self):
            # This pulls the 'best' message based on your YAML priorities
            ready_to_send = self.dynamic_buffer.pop()
            
            if ready_to_send:
                out_msg = ByteMultiArray()
                out_msg.data = list(ready_to_send)
                self.pub.publish(out_msg)



def main(args=None):
    rclpy.init(args=args)
    node = TrafficControlRos()
    rclpy.spin(node)  # Keep the node running to allow timer execution
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()