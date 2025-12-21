import rclpy
from rclpy.node import Node
import threading
import time

from include.dynamic_buffer import DynamicBufferPython
from std_msgs.msg import ByteMultiArray


class TrafficControlRos(Node):
    def __init__(self):
        super().__init__('mvp_c2_traffic_control')
        self.dccl_intake_sub = self.create_subscription(ByteMultiArray, 
                                                    'mvp_c2/dccl_msg_tx', 
                                                    self.dccl_tx_callback, 10)
        self.dynamic_buffer = DynamicBufferPython(max_total_size=50)
        print("Initialization done", flush= True)


    def dccl_tx_callback():
        print("Good")
        #add data into dynamic buffer
            

def main(args=None):
    rclpy.init(args=args)
    node = TrafficControlRos()
    rclpy.spin(node)  # Keep the node running to allow timer execution
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
