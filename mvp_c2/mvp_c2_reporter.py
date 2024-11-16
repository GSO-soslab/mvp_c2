import rclpy
import sys, os
import dccl
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Trigger
from geographic_msgs.msg import GeoPoseStamped
# from mvp_msgs.srv import GetControlMode

from dccl_checksum import check_dccl, package_dccl

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

        self.geopose_sub = self.create_subscription(GeoPoseStamped, 'geopose', self.geopose_callback, 10)
        
        #client
        self.report_controller_client = self.create_client(Trigger, 'controller/get_state')

        #DCCL
        self.ddcl_reporter_pub = self.create_publisher(UInt8MultiArray, 'reporter/dccl_msg', 10)
        dccl.loadProtoFile(os.path.join( get_package_share_directory(package_name), 
                                         'proto', 
                                         'mvp_cmd_dccl.proto') )
        self.dccl_obj = dccl.Codec()
        self.source = 1
        self.destination =2
        print("reporter initialized", flush=True)

    ##publish dccl 
    def publish_dccl(self, proto):
        dccl_msg = UInt8MultiArray()
        dccl_msg.data = self.dccl_obj.encode(proto)
        dccl_msg.data = package_dccl(dccl_msg.data)
        self.ddcl_reporter_pub.publish(dccl_msg)
        return True

    #odometry callback
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
        
        proto.orientation.extend([ msg.pose.pose.orientation.x, 
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w ])
        
        proto.uvw.extend([ msg.twist.twist.linear.x,
                           msg.twist.twist.linear.y, 
                           msg.twist.twist.linear.z ]) 
        
        proto.pqr.extend([ msg.twist.twist.angular.x,
                           msg.twist.twist.angular.y, 
                           msg.twist.twist.angular.z ]) 
        
        self.publish_dccl(proto)

    #geopose callback
    def geopose_callback(self, msg):
        print("got geopose")
        self.dccl_obj.load('GeoPose')
        proto = mvp_cmd_dccl_pb2.GeoPose()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.source = self.source
        proto.destination = self.destination
        proto.lla.extend([ msg.pose.position.latitude, 
                            msg.pose.position.lonhitude,
                            msg.pose.position.altitude ])
        
        proto.orientation.extend([ msg.pose.orientation.x, 
                                msg.pose.orientation.y,
                                msg.pose.orientation.z,
                                msg.pose.orientation.w ])

        self.publish_dccl(proto)

    #report controller callback
    def report_controller_callback(self):
        while not self.report_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to become available...')
        
        request = Trigger.Request()
        future = self.report_controller_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            # self.get_logger().info(f'Response: {future.result().success}, Message: {future.result().message}')
            print("got controller state")
            self.dccl_obj.load('ReportController')
            proto = mvp_cmd_dccl_pb2.ReportController()
            proto.time =round(time.time(), 3)
            proto.source = self.source
            proto.destination = self.destination
            
            response = future.result()
            # Check response message and print 0 or 1
            if response.message == "enabled":
                proto.status = 1
            elif response.message == "disabled":
                proto.status = 0

            #publish dccl msg
            self.publish_dccl(proto)
            
        else:
            self.get_logger().error('Service call failed')
        
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