import rclpy
import sys, os
import dccl
import signal
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray, Bool
from sensor_msgs.msg import Joy
from mvp_msgs.srv import SetString
from mvp_msgs.srv import ChangeState, GetState
from mvp_msgs.msg import HelmState

from std_srvs.srv import Trigger, SetBool
from geographic_msgs.msg import GeoPoseStamped
# from mvp_msgs.srv import GetControlMode

from include.dccl_checksum import check_dccl, package_dccl
from include.roslaunch_manager import ROSLaunchManager

# sys.path.append('../proto')  # Adjust path if needed
import mvp_cmd_dccl_pb2
import time 
from ament_index_python.packages import get_package_share_directory

package_name = 'mvp_c2'

# Full path to the config file



class MvpC2Reporter(Node):

    def __init__(self):
        super().__init__('mvp_c2_reporter')

        # ns = self.get_namespace()

        self.local_id = self.declare_parameter('local_id', 2).value
        self.remote_id = self.declare_parameter('remote_id', 1).value
        self.dccl_tx_interval = self.declare_parameter('dccl_tx_interval', 2.0).value
        self.local_mvp_active = self.declare_parameter('local_mvp_active', True).value
        self.ser_wait_time = self.declare_parameter('service_wait_time', 1.0).value

        
        # self.default_state_list = ['start', 'kill', 'survey', 'profiling', 'teleop']
        # self.launch_packages = ['mvp2_test_robot_bringup']
        # self.launch_file_names = ['bringup_simulation']

        self.declare_parameter('helm_state_list', [''])
        self.default_state_list = self.get_parameter('helm_state_list').get_parameter_value().string_array_value
        
        self.declare_parameter('launch_packages', [''])
        self.launch_packages = self.get_parameter('launch_packages').get_parameter_value().string_array_value
     
        self.declare_parameter('launch_files', [''])
        self.launch_file_names = self.get_parameter('launch_files').get_parameter_value().string_array_value

        ##roslauncher
        self.roslauncher = ROSLaunchManager()

        # mvp_active meaning the local machine has mvp running so it can transfer its mvp related 
        # data to the remote machine
        #susbcribe to different topics running on the source
        
        self.local_odom_sub = self.create_subscription(Odometry, 'local/odometry', self.odom_callback, 10)
        self.local_geopose_sub = self.create_subscription(GeoPoseStamped, 'local/geopose', self.geopose_callback, 10)

        #client for local controllers
        self.local_set_controller_client = self.create_client(SetBool, 'controller/set')
        self.local_report_controller_client = self.create_client(Trigger, 'controller/get_state')
        self.local_set_helm_client = self.create_client(ChangeState, 'mvp_helm/change_state')
        self.local_report_helm_client = self.create_client(GetState, 'mvp_helm/get_state')

        #joy stick publisher from base station
        self.local_joy_pub = self.create_publisher(Joy, 'joy', 10)

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
        self.local_report_controller_state_tx_flag = False
        self.local_report_helm_state_tx_flag = False

        self.timer = self.create_timer(self.dccl_tx_interval, self.reset_dccl_tx_flag)
        self.timer2 = self.create_timer(self.dccl_tx_interval, self.report_controller_state_callback)
        self.timer3 = self.create_timer(self.dccl_tx_interval, self.report_helm_state_callback)

    def reset_dccl_tx_flag(self):
        self.local_odom_tx_flag = False
        self.local_geopose_tx_flag = False
        self.local_report_controller_state_tx_flag = False
        self.local_report_helm_state_tx_flag = False

    #######################################################
    ############DCCL parsing###############################
    #######################################################
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

                    self.local_joy_pub.publish(msg)
                except Exception as e:
                    # Print the exception message for debugging
                    print(f"Decoding error: {e}", flush=True)

            #set controller
            if message_id ==22:
                try: 
                    self.dccl_obj.load('SetController')
                    proto_msg = self.dccl_obj.decode(data)
                    # print(proto_msg, flush = True)
                    self.local_set_controller_client.wait_for_service(timeout_sec=self.ser_wait_time)
                    # self.get_logger().info(
                    #         f"Waiting for service '{self.local_set_controller_client.srv_name}' to become available..."
                    #     )

                    request = SetBool.Request()
                    request.data = proto_msg.status

                    future = self.local_set_controller_client.call_async(request)
                    # rclpy.spin_until_future_complete(self, future)

                except Exception as e:
                    # Print the exception message for debugging
                    print(f"Decoding error: {e}", flush=True)

            ##change helm state
            if message_id == 30:
                # print("got change helm ", flush = True)
                try:
                    self.dccl_obj.load('SetHelm')
                    proto_msg = self.dccl_obj.decode(data)
                    # print(proto_msg, flush = True)
                    self.local_set_helm_client.wait_for_service(timeout_sec=self.ser_wait_time)
                    # self.get_logger().info(
                    #         f"Waiting for service '{self.local_set_helm_client.srv_name}' to become available..."
                    #     )

                    request = ChangeState.Request()
                    request.state = self.default_state_list[proto_msg.state]
                    # print(request.state, flush = True)
                    request.caller = "dccl"
                    future = self.local_set_helm_client.call_async(request)
                    # rclpy.spin_until_future_complete(self, future)
                except Exception as e:
                    # Print the exception message for debugging
                    print(f"Decoding error: {e}", flush=True)

            #roslaunch request
            if message_id == 40: 
                try:
                    self.dccl_obj.load('RosLaunch')
                    proto_msg = self.dccl_obj.decode(data)
                    index = proto_msg.index
                    req = proto_msg.req
                    print(f"{self.launch_packages[index]}/{self.launch_file_names[index]} | set to {req}")
                    if req == True:
                        self.roslauncher.start_launch(self.launch_packages[index], self.launch_file_names[index])
                    else:
                        self.roslauncher.stop_launch(self.launch_packages[index], self.launch_file_names[index])
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
    
    #######################################################
    ############Callback###################################
    #######################################################
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
        print
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

    #report controller backback
    def report_controller_state_callback(self):
        # Block until the service is available (1-second timeout)
        try:
            self.local_report_controller_client.wait_for_service(timeout_sec=self.ser_wait_time)
            # self.get_logger().info(f"Waiting for service '{self.local_report_controller_client.srv_name}' to become available...")
            request = Trigger.Request()
            future = self.local_report_controller_client.call_async(request)

            # Handle the response
            future.add_done_callback(self.report_controller_state_callback_done)
        except:
                print("Get Controller State Service Timeout")

    def report_controller_state_callback_done(self, future):
        response = future.result()
        # self.get_logger().info(f'Service response: {response.message}')
        ##make dccl
        self.dccl_obj.load('ReportController')
        proto = mvp_cmd_dccl_pb2.ReportController()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.status = response.message == "enabled"
        # print(proto, flush = True)
        if self.local_report_controller_state_tx_flag is False:
            self.publish_dccl(proto)
            self.local_report_controller_state_tx_flag = True
        return response      


    def report_helm_state_callback(self):
        # Block until the service is available (1-second timeout)
        try:
            self.local_report_helm_client.wait_for_service(timeout_sec=self.ser_wait_time)
            # self.get_logger().info(f"Waiting for service '{self.local_report_helm_client.srv_name}' to become available...")

            request = GetState.Request()
            request.name = ''
            future = self.local_report_helm_client.call_async(request)

            # Handle the response
            future.add_done_callback(self.report_helm_state_callback_done)
        except:
                print("Get Helm State Service Timeout")

    def report_helm_state_callback_done(self, future):
        response = future.result()
        # self.get_logger().info(f'Service response: {response}')
        ##make dccl
        self.dccl_obj.load('ReportHelm')
        proto = mvp_cmd_dccl_pb2.ReportHelm()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        # proto.state = response.state.name
        
        if response.state.name in self.default_state_list:
            index = self.default_state_list.index(response.state.name)
        else:
            print(f"'{response.state_name}' not found in default_state_list")
            index = 0
        proto.state = index
        
        indices = [self.default_state_list.index(transition) 
           for transition in response.state.transitions if transition in self.default_state_list]
        proto.connected_state.extend(indices) 

        # print(proto, flush = True)
        if self.local_report_helm_state_tx_flag is False:
            self.publish_dccl(proto)
            # print(proto, flush = True)
            self.local_report_helm_state_tx_flag = True
        return response   
 

def main(args=None):
    rclpy.init(args=args)

    node = MvpC2Reporter()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()