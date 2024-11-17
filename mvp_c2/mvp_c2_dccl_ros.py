import rclpy
import sys, os
import dccl
import signal
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Joy

from std_srvs.srv import Trigger, SetBool
from geographic_msgs.msg import GeoPoseStamped
# from mvp_msgs.srv import GetControlMode

from dccl_checksum import check_dccl, package_dccl

# sys.path.append('../proto')  # Adjust path if needed
import mvp_cmd_dccl_pb2
import time 
from ament_index_python.packages import get_package_share_directory

package_name = 'mvp_c2'

# Full path to the config file



class MvpC2Dccl(Node):

    def __init__(self):
        super().__init__('mvp_c2_dccl')

        # ns = self.get_namespace()

        self.local_id = self.declare_parameter('local_id', 1).value
        self.remote_id = self.declare_parameter('remote_id', 2).value
        self.dccl_tx_interval = self.declare_parameter('dccl_tx_interval', 2.0).value
        self.machine_type = self.declare_parameter('machine_type', 'topside').value #robot

        #susbcribe to different topics running on the source
        self.local_odom_sub = self.create_subscription(Odometry, 'local/odometry', self.odom_callback, 10)
        self.local_geopose_sub = self.create_subscription(GeoPoseStamped, 'local/geopose', self.geopose_callback, 10)
        self.local_joy_sub = self.create_subscription(Joy, 'local/joy', self.joy_callback, 10)

        ##publish information parsed from dccl to ros topic
        self.remote_odom_pub = self.create_publisher(Odometry, 'remote/odometry', 10)
        self.remote_geopose_pub = self.create_publisher(GeoPoseStamped, 'remote/geopose', 10)
        self.remote_joy_pub = self.create_publisher(Joy, 'remote/joy', 10)

        ##service for access remote controllers
        self.remote_set_controller_srv = self.create_service(SetBool, 'remote/controller/set', self.remote_set_controller_callback)
        # self.remote_get_controller_srv = self.create_service(Trigger, '~/remote/controller/get_state', self.remote_get_controller_callback)

        #client for local controllers
        self.local_set_controller_client = self.create_client(SetBool, 'controller/set')


        #local controller status will be checked using timer and ->dccl tx
        # self.local_get_controller_client = 

        #DCCL byte array topic
        self.ddcl_reporter_pub = self.create_publisher(UInt8MultiArray, 'mvp_c2/dccl_msg_tx', 10)

        self.dccl_reporter_sub = self.create_subscription(UInt8MultiArray, 
                                                        'mvp_c2/dccl_msg_rx', 
                                                        self.dccl_rx_callback, 10)


        dccl.loadProtoFile(os.path.join( get_package_share_directory(package_name), 
                                         'proto', 
                                         'mvp_cmd_dccl.proto') )
        
        self.dccl_obj = dccl.Codec()
        print("dccl_ros_node initialized", flush=True)

        ##timer for resetting the dccl tx flag
        self.local_odom_tx_flag = False
        self.local_geopose_tx_flag = False
        self.local_joy_tx_flag = False

        self.timer = self.create_timer(self.dccl_tx_interval, self.reset_dccl_tx_flag)


    def reset_dccl_tx_flag(self):
        self.local_odom_tx_flag = False
        self.local_geopose_tx_flag = False
        self.local_joy_tx_flag = False
        self.remote_set_controller_tx_flag = False


    ###parsing dccl
    def dccl_rx_callback(self,msg):
        # print("got dccl", flush=True)
        flag, data = check_dccl(msg.data)
        if flag == True:
            message_id = self.dccl_obj.id(data)
            # print(message_id, flush = True)
            #Joy
            if message_id == 1:
                try:
                    self.dccl_obj.load('Joy')
                    proto_msg = self.dccl_obj.decode(data)
                    # print(decoded_msg, flush = True)
                    msg = Joy()
                    sec = int(proto_msg.time)  
                    nanosec = int((proto_msg.time - sec) * 1e9)  
                    msg.header.stamp.sec = sec
                    msg.header.stamp.nanosec = nanosec
                    msg.axes = proto_msg.axes
                    msg.buttons = proto_msg.buttons

                    self.remote_joy_pub.publish(msg)
                except Exception as e:
                    # Print the exception message for debugging
                    print(f"Decoding error: {e}", flush=True)

            #odometry 
            if message_id == 3:
                try:
                    self.dccl_obj.load('Odometry')
                    proto_msg = self.dccl_obj.decode(data)
                    # print(decoded_msg, flush = True)
                    msg = Odometry()
                    sec = int(proto_msg.time)  
                    nanosec = int((proto_msg.time - sec) * 1e9)  
                    msg.header.stamp.sec = sec
                    msg.header.stamp.nanosec = nanosec
                    msg.header.frame_id = proto_msg.frame_id
                    msg.child_frame_id = proto_msg.child_frame_id
                    #map position
                    if len(proto_msg.position) ==3:
                        msg.pose.pose.position.x = proto_msg.position[0]
                        msg.pose.pose.position.y = proto_msg.position[1]
                        msg.pose.pose.position.z = proto_msg.position[2]

                    if len(proto_msg.orientation) ==4:
                        msg.pose.pose.orientation.x = proto_msg.orientation[0]
                        msg.pose.pose.orientation.y = proto_msg.orientation[1]
                        msg.pose.pose.orientation.z = proto_msg.orientation[2]
                        msg.pose.pose.orientation.w = proto_msg.orientation[3]

                    if len(proto_msg.uvw) ==3:
                        msg.twist.twist.linear.x = proto_msg.uvw[0]
                        msg.twist.twist.linear.y = proto_msg.uvw[1]
                        msg.twist.twist.linear.z = proto_msg.uvw[2]

                    if len(proto_msg.pqr) ==3:
                        msg.twist.twist.angular.x = proto_msg.pqr[0]
                        msg.twist.twist.angular.y = proto_msg.pqr[1]
                        msg.twist.twist.angular.z = proto_msg.pqr[2]

                    self.remote_odom_pub.publish(msg)
                except Exception as e:
                    # Print the exception message for debugging
                    print(f"Decoding error: {e}", flush=True)

            #geopose
            if message_id == 4:
                try:
                    self.dccl_obj.load('GeoPose')
                    proto_msg = self.dccl_obj.decode(data)
                    # print(decoded_msg, flush = True)
                    msg = GeoPoseStamped()
                    sec = int(proto_msg.time)  
                    nanosec = int((proto_msg.time - sec) * 1e9)  
                    msg.header.stamp.sec = sec
                    msg.header.stamp.nanosec = nanosec
                    msg.header.frame_id = proto_msg.frame_id
                    if len(proto_msg.lla) ==3:
                        msg.pose.position.latitude = proto_msg.lla[0]
                        msg.pose.position.longitude = proto_msg.lla[1]
                        msg.pose.position.altitude = proto_msg.lla[2]
                    if len(proto_msg.orientation) ==4:
                        msg.pose.orientation.x = proto_msg.orientation[0]
                        msg.pose.orientation.y = proto_msg.orientation[1]
                        msg.pose.orientation.z = proto_msg.orientation[2]
                        msg.pose.orientation.w = proto_msg.orientation[3]

                    self.remote_geopose_pub.publish(msg)
                except Exception as e:
                    # Print the exception message for debugging
                    print(f"Decoding error: {e}", flush=True)
            
            #set controller
            if message_id ==22:
                try: 
                    self.dccl_obj.load('SetController')
                    proto_msg = self.dccl_obj.decode(data)
                    # print(proto_msg, flush = True)
                    while not self.local_set_controller_client.wait_for_service(timeout_sec=1.0):
                       self.get_logger().info(
                            f"Waiting for service '{self.local_set_controller_client.srv_name}' to become available..."
                        )

                    request = SetBool.Request()
                    request.data = proto_msg.status

                    future = self.local_set_controller_client.call_async(request)
                    rclpy.spin_until_future_complete(self, future)

                except Exception as e:
                    # Print the exception message for debugging
                    print(f"Decoding error: {e}", flush=True)


    ##publish dccl 
    def publish_dccl(self, proto):
        dccl_msg = UInt8MultiArray()
        dccl_msg.data = self.dccl_obj.encode(proto)
        dccl_msg.data = package_dccl(dccl_msg.data)
        self.ddcl_reporter_pub.publish(dccl_msg)
        return True

    #odometry callback
    def odom_callback(self, msg):
        # print("got odometry", flush =True)
        self.dccl_obj.load('Odometry')
        proto = mvp_cmd_dccl_pb2.Odometry()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
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
        
        proto.frame_id = msg.header.frame_id
        proto.child_frame_id = msg.child_frame_id
        
        if self.local_odom_tx_flag is False:
            self.publish_dccl(proto)
            self.local_odom_tx_flag = True

    #geopose callback
    def geopose_callback(self, msg):
        # print("got geopose")
        self.dccl_obj.load('GeoPose')
        proto = mvp_cmd_dccl_pb2.GeoPose()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.lla.extend([ msg.pose.position.latitude, 
                            msg.pose.position.longitude,
                            msg.pose.position.altitude ])
        
        proto.orientation.extend([ msg.pose.orientation.x, 
                                msg.pose.orientation.y,
                                msg.pose.orientation.z,
                                msg.pose.orientation.w ])
        
        proto.frame_id = msg.header.frame_id
    
        if self.local_geopose_tx_flag is False:
            self.publish_dccl(proto)
            self.local_geopose_tx_flag = True

    #joy callback
    def joy_callback(self, msg):
        # print("got joy")
        self.dccl_obj.load('Joy')
        proto = mvp_cmd_dccl_pb2.Joy()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.axes.extend(msg.axes)  # Map axes
        proto.buttons.extend(msg.buttons)  # Map buttons
        if self.local_joy_tx_flag is False:
            self.publish_dccl(proto)
            self.local_joy_tx_flag = True

    #set remote controller service callback
    def remote_set_controller_callback(self, request, response):
        # print("got set controller")
        self.dccl_obj.load('SetController')
        proto = mvp_cmd_dccl_pb2.SetController()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.status = request.data
        response.success = True
        response.message = 'Service called'
        if self.remote_set_controller_tx_flag is False:
            self.publish_dccl(proto)
            self.remote_set_controller_tx_flag = True
        return response        
        
        # if future.result() is not None:
    #         # self.get_logger().info(f'Response: {future.result().success}, Message: {future.result().message}')
    #         print("got controller state")
    #         self.dccl_obj.load('ReportController')
    #         proto = mvp_cmd_dccl_pb2.ReportController()
    #         proto.time =round(time.time(), 3)
    #         proto.local_id = self.local_id
    #         proto.remote_id = self.remote_id
            
    #         response = future.result()
    #         # Check response message and print 0 or 1
    #         if response.message == "enabled":
    #             proto.status = 1
    #         elif response.message == "disabled":
    #             proto.status = 0

    #         #publish dccl msg
    #         self.publish_dccl(proto)
            
    #     else:
    #         self.get_logger().error('Service call failed')
        
def main(args=None):
    rclpy.init(args=args)

    node = MvpC2Dccl()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()