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



class MvpC2Reporter(Node):

    def __init__(self):
        super().__init__('mvp_c2_reporter')
    
        #susbcribe to different topics
        self.odom_sub = self.create_subscription(Odometry, 'odometry', self.odom_callback, 10)

        #DCCL
        self.ddcl_reporter_pub = self.create_publisher(UInt8MultiArray, 'reporter/dccl_msg', 10)
        dccl.loadProtoFile(os.path.join( get_package_share_directory(package_name), 
                                         'proto', 
                                         'mvp_cmd_dccl.proto') )
        self.dccl_obj = dccl.Codec()
        self.source = 1
        self.destination =2
        print("reporter initialized", flush=True)

    def package_dccl(self, data):
        # Calculate the checksum for the NMEA string
        checksum = 0
        for byte in data:
            checksum ^= byte     
        ender_bytes = f"*{checksum:02X}"
        data_out = bytearray('$$$', 'ascii') + data +  bytearray(ender_bytes+'\n', 'ascii')
        return data_out

    def odom_callback(self, msg):
        print("got odometry")
        self.dccl_obj.load('Odometry')
        proto = mvp_cmd_dccl_pb2.Odometry()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.source = self.source
        proto.destination = self.destination
        proto.position.extend([ msg.pose.pose.position.x, 
                                msg.pose.pose.position.y,
                                msg.pose.pose.position.z ])
        
        proto.position.extend([ msg.pose.pose.orientation.x, 
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w ])
        
        proto.uvw.extend([ msg.twist.twist.linear.x,
                           msg.twist.twist.linear.y, 
                           msg.twist.twist.linear.z ]) 
        
        proto.pqr.extend([ msg.twist.twist.angular.x,
                           msg.twist.twist.angular.y, 
                           msg.twist.twist.angular.z ]) 
        
        dccl_msg = UInt8MultiArray()
        dccl_msg.data = self.dccl_obj.encode(proto)
        dccl_msg.data = self.package_dccl(dccl_msg.data)
        # print(dccl_msg.data, flush = True)
        # print(len(dccl_msg.data), flush = True)
        self.ddcl_reporter_pub.publish(dccl_msg)


def main(args=None):
    rclpy.init(args=args)

    reporter_node = MvpC2Reporter()

    rclpy.spin(reporter_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reporter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()