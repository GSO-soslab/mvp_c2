import rclpy
import sys, os
import dccl
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray

# sys.path.append('../proto')  # Adjust path if needed
import mvp_cmd_dccl_pb2
import time 
from ament_index_python.packages import get_package_share_directory

package_name = 'mvp_c2'

# Full path to the config file



class MvpC2Commander(Node):

    def __init__(self):
        super().__init__('mvp_c2_commander')
        #susbcribe to different topics
        self.dccl_reporter_sub = self.create_subscription(UInt8MultiArray, 
                                                        'reporter/dccl_msg', 
                                                        self.dccl_reporter_callback, 10)

        #DCCL
        dccl.loadProtoFile(os.path.join( get_package_share_directory(package_name), 
                                         'proto', 
                                         'mvp_cmd_dccl.proto') )
        self.dccl_obj = dccl.Codec()
        self.source = 2
        self.destination =1
        print('commander initialized', flush=True)

    def dccl_reporter_callback(self, msg):
        # print("got dccl", flush=True)
        byte_array = bytes(msg.data)
        message_id = self.dccl_obj.id(byte_array)
        print(message_id, flush = True)
        # try:
        #     # self.dccl_obj.load('Odometry')
            
        #     # decoded_msg = self.dccl_obj.decode(byte_array)
            
           
        # except Exception as e:
        #     # Print the exception message for debugging
        #     print(f"Decoding error: {e}", flush=True)

def main(args=None):
    rclpy.init(args=args)

    commader_node = MvpC2Commander()

    rclpy.spin(commader_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    commader_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()