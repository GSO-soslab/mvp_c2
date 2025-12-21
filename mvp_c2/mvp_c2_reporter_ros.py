import rclpy
import sys, os
import dccl
import signal
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, ByteMultiArray, Float32MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PointStamped
# from mvp_msgs.srv import SetString
from mvp_msgs.srv import ChangeState, GetState, GetWaypoints, SendWaypoints
from mvp_msgs.msg import Waypoint

from std_srvs.srv import Trigger, SetBool
from geographic_msgs.msg import GeoPoseStamped, GeoPointStamped
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
        self.ser_wait_time = self.declare_parameter('service_wait_time', 0.2).value

        self.declare_parameter('helm_state_list', [''])
        self.default_state_list = self.get_parameter('helm_state_list').get_parameter_value().string_array_value
        
        self.declare_parameter('launch_packages', [])
        self.launch_packages = self.get_parameter('launch_packages').get_parameter_value().string_array_value
     
        self.declare_parameter('launch_files', [])
        self.launch_file_names = self.get_parameter('launch_files').get_parameter_value().string_array_value

        self.declare_parameter('gpio_devices', [])
        self.gpio_devices = self.get_parameter('gpio_devices').get_parameter_value().string_array_value

        ##roslauncher
        self.roslauncher = ROSLaunchManager()

        # mvp_active meaning the local machine has mvp running so it can transfer its mvp related 
        # data to the remote machine
        #susbcribe to different topics running on the source
        
        self.local_odom_sub = self.create_subscription(Odometry, 'local/odometry', self.odom_callback, 10)
        self.local_geopose_sub = self.create_subscription(GeoPoseStamped, 'local/geopose', self.geopose_callback, 10)
        self.local_acomm_geopoint_sub = self.create_subscription(GeoPointStamped, 'local/acomm_geopoint', self.acomm_geopoint_callback, 10)
        self.power_vi_sub = self.create_subscription(Float32MultiArray, 'local/power_monitor', self.power_vi_callback,10)
        self.cpu_info_sub = self.create_subscription(Float32MultiArray, 'local/computer_info', self.cpu_info_callback,10)
        self.altimeter_sub = self.create_subscription(PointStamped, 'local/altimeter', self.altimeter_callback, 10)


        #client for local controllers
        self.local_set_controller_client = self.create_client(SetBool, 'controller/set')
        self.local_report_controller_client = self.create_client(Trigger, 'controller/get_state')
        self.local_set_helm_client = self.create_client(ChangeState, 'mvp_helm/change_state')
        self.local_report_helm_client = self.create_client(GetState, 'mvp_helm/get_state')
        self.local_report_wpt_client = self.create_client(GetWaypoints, 'mvp_helm/path')
        self.local_set_wpt_client = self.create_client(SendWaypoints, 'mvp_helm/set_waypoints')
        self.local_report_gpio_client = self.create_client(Trigger, 'gpio_manager/get_power_status')
        self.local_set_gpio_clients = {}
        self.local_reset_datum_client = self.create_client(Trigger, 'reset_datum')
        

        for index in range(len(self.gpio_devices)):
            srv_name = 'gpio_manager/set_power/' + self.gpio_devices[index]
            client = self.create_client(SetBool,srv_name)
            self.local_set_gpio_clients[index] = client
            self.get_logger().info(f"Created client for {srv_name}")

        #joy stick publisher from base station
        self.local_joy_pub = self.create_publisher(Joy, 'joy', 10)

        #DCCL byte array topic
        self.ddcl_reporter_pub = self.create_publisher(ByteMultiArray, 'mvp_c2/reporter/dccl_msg_tx', 10)
        
        self.dccl_reporter_sub = self.create_subscription(ByteMultiArray, 
                                                        'mvp_c2/reporter/dccl_msg_rx', 
                                                        self.dccl_rx_callback, 10)


        dccl.loadProtoFile(os.path.join( get_package_share_directory(package_name), 
                                         'proto', 
                                         'mvp_cmd_dccl.proto') )
        
        self.dccl_obj = dccl.Codec()
        print("dccl_ros_node initialized", flush=True)
        self.load_dccl()
        ##timer for resetting the dccl tx flag
        # self.local_odom_tx_flag = False
        # self.local_geopose_tx_flag = False
        # self.local_acomm_geopoint_tx_flag = False

        # self.local_report_controller_state_tx_flag = False
        # self.local_report_helm_state_tx_flag = False
        # self.local_report_wpt_tx_flag = False
        # self.local_report_roslaunch_tx_flag = False
        # self.local_report_gpio_tx_flag = False
        # self.local_power_info_tx_flag = False
        # self.local_cpu_info_tx_flag = False
        # self.local_altimeter_info_tx_flag = False



        # self.timer = self.create_timer(self.dccl_tx_interval, self.reset_dccl_tx_flag)
        self.timer2 = self.create_timer(self.dccl_tx_interval, self.report_controller_state_callback)
        self.timer3 = self.create_timer(self.dccl_tx_interval, self.report_helm_state_callback)
        self.timer4 = self.create_timer(self.dccl_tx_interval, self.report_wpt_callback)
        self.timer4 = self.create_timer(self.dccl_tx_interval, self.report_roslaunch_callback)
        self.timer5 = self.create_timer(self.dccl_tx_interval, self.report_gpio_callback)


    # def reset_dccl_tx_flag(self):
    #     self.local_odom_tx_flag = False
    #     self.local_geopose_tx_flag = False
    #     self.local_acomm_geopoint_tx_flag = False

    #     self.local_report_controller_state_tx_flag = False
    #     self.local_report_helm_state_tx_flag = False
    #     self.local_report_wpt_tx_flag = False
    #     self.local_report_roslaunch_tx_flag = False
    #     self.local_report_gpio_tx_flag = False
    #     self.local_power_info_tx_flag = False
    #     self.local_cpu_info_tx_flag = False
    #     self.local_altimeter_info_tx_flag = False


    def load_dccl(self):
        self.dccl_obj.load('Joy')
        self.dccl_obj.load('PWM')
        self.dccl_obj.load('Odometry')
        self.dccl_obj.load('GeoPose')
        self.dccl_obj.load('PowerMonitor')
        self.dccl_obj.load('CPUMonitor')
        self.dccl_obj.load('AltimeterPointStamped')
        self.dccl_obj.load('AcommGeoPoint')
        self.dccl_obj.load('SetPowerPort')
        self.dccl_obj.load('ReportPowerPort')
        self.dccl_obj.load('SetController')
        self.dccl_obj.load('ReportController')
        self.dccl_obj.load('SetHelm')
        self.dccl_obj.load('ReportHelm')
        self.dccl_obj.load('SetWpt')
        self.dccl_obj.load('ReportWpt')
        self.dccl_obj.load('ResetDatum')
        self.dccl_obj.load('RosLaunch')
        self.dccl_obj.load('ReportRosLaunch')


    #######################################################
    ############DCCL parsing###############################
    #######################################################
    ###parsing dccl
    def dccl_rx_callback(self,msg):
        # print("got dccl", flush=True)
        bdata = bytearray(ord(c) for c in msg.data)
        flag, data = check_dccl(bdata)
        # flag, data = check_dccl(msg.data)

        if flag == True:
            message_id = self.dccl_obj.id(data)
            print(f'{round(time.time(), 3)}: dccl_message_id: {message_id}, data_len: {len(data)}', flush=True)

            # print(message_id, flush = True)
            #Joy
            if message_id == 1:
                try:
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
                    proto_msg = self.dccl_obj.decode(data)
                    self.local_set_controller_client.wait_for_service(timeout_sec=self.ser_wait_time)
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
                    proto_msg = self.dccl_obj.decode(data)
                    # print(proto_msg, flush = True)
                    self.local_set_helm_client.wait_for_service(timeout_sec=self.ser_wait_time)

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
                    proto_msg = self.dccl_obj.decode(data)
                    index = proto_msg.index
                    req = proto_msg.req
                    print(f"{self.launch_packages[index]}/{self.launch_file_names[index]} | set to {req}", flush = True)
                    if req == True:
                        self.roslauncher.start_launch(self.launch_packages[index], self.launch_file_names[index])
                    else:
                        self.roslauncher.stop_launch(self.launch_packages[index], self.launch_file_names[index])
                except Exception as e:
                    # Print the exception message for debugging
                    print(f"Decoding error: {e}", flush=True)

            #set power request
            if message_id == 20:
                try:
                    proto_msg = self.dccl_obj.decode(data)
                    index = proto_msg.index
                    request = SetBool.Request()
                    request.data = proto_msg.state
                    future = self.local_set_gpio_clients[index].call_async(request)
                    print(f"{self.gpio_devices[index]} Power set to {request.data}", flush =True)
                    
                except Exception as e:
                    # Print the exception message for debugging
                    print(f"Decoding error: {e}", flush=True)

            ##waypoint set dccl
            if message_id == 32:
                try:
                    proto_msg = self.dccl_obj.decode(data)
                    self.local_set_wpt_client.wait_for_service(timeout_sec=self.ser_wait_time)

                    request = SendWaypoints.Request()  
                    request.type = 'geopath'

                    request.wpt = [Waypoint() for _ in range(proto_msg.wpt_size)]
                    for i in range(proto_msg.wpt_size):
                        request.wpt[i].ll_wpt.latitude = proto_msg.latitude[i]*0.01
                        request.wpt[i].ll_wpt.longitude = proto_msg.longitude[i]*0.01
                        request.wpt[i].ll_wpt.altitude = proto_msg.altitude[i]
                        request.wpt[i].u = proto_msg.u[i]
                    future = self.local_set_wpt_client.call_async(request)
                except Exception as e:
                    # Print the exception message for debugging
                    print(f"Decoding error: {e}", flush=True)
            
            ##reset datum
            if message_id == 34:
                try:
                    proto_msg = self.dccl_obj.decode(data)
                    self.local_reset_datum_client.wait_for_service(timeout_sec=self.ser_wait_time)
                    request = Trigger.Request()  
                    future = self.local_reset_datum_client.call_async(request)
                    print("Datum reset triggered", flush =True)
                except Exception as e:
                    # Print the exception message for debugging
                    print(f"Decoding error: {e}", flush=True)
    ##publish dccl 
    def publish_dccl(self, proto):
        dccl_msg = ByteMultiArray()
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
        # proto.frame_id = msg.header.frame_id
        # proto.child_frame_id = msg.child_frame_id
        
        # if self.local_odom_tx_flag is False:
        self.publish_dccl(proto)
            # self.local_odom_tx_flag = True

    #geopose callback
    def geopose_callback(self, msg):
        # print("got geopose")
        proto = mvp_cmd_dccl_pb2.GeoPose()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.latitude = msg.pose.position.latitude*100
        proto.longitude = msg.pose.position.longitude*100
        proto.altitude = msg.pose.position.altitude
        
        proto.orientation.extend([ msg.pose.orientation.x, 
                                msg.pose.orientation.y,
                                msg.pose.orientation.z,
                                msg.pose.orientation.w ])
        
        # proto.frame_id = msg.header.frame_id
    
        # if self.local_geopose_tx_flag is False:
        self.publish_dccl(proto)
            # self.local_geopose_tx_flag = True

    #acomm geopose topic (from usbl)
    def acomm_geopoint_callback(self, msg):
        # print("got acom_geopose")
        proto = mvp_cmd_dccl_pb2.AcommGeoPoint()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.latitude = msg.position.latitude*100
        proto.longitude = msg.position.longitude*100
        proto.altitude = msg.position.altitude
            
        # if self.local_acomm_geopoint_tx_flag is False:
        self.publish_dccl(proto)
            # self.local_acomm_geopoint_tx_flag = True

    #power monitor
    def power_vi_callback(self, msg):
        proto = mvp_cmd_dccl_pb2.PowerMonitor()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.data.extend([msg.data[0], msg.data[1]])
        # if self.local_power_info_tx_flag is False:
        self.publish_dccl(proto)
            # self.local_power_info_tx_flag = True
    
    #cpu monitor
    def cpu_info_callback(self, msg):
        proto = mvp_cmd_dccl_pb2.CPUMonitor()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.data.extend([ msg.data[0], msg.data[1], msg.data[2]])
        # if self.local_cpu_info_tx_flag is False:
        self.publish_dccl(proto)
            # self.local_cpu_info_tx_flag = True

    #altimeter
    def altimeter_callback(self, msg):
        proto = mvp_cmd_dccl_pb2.AltimeterPointStamped()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.data = msg.point.z
        # if self.local_altimeter_info_tx_flag is False:
        self.publish_dccl(proto)
            # self.local_altimeter_info_tx_flag = True

    def report_roslaunch_callback(self):
        # if(self.local_report_roslaunch_tx_flag == False):

        running_launches = self.roslauncher.list_running_launches()
        # data = []
        proto = mvp_cmd_dccl_pb2.ReportRosLaunch()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id

        for index, name in enumerate(self.launch_file_names):
            key = (self.launch_packages[index], name)
            if key in running_launches:
                proto.state.append(1)
                # data.append(True)
            else:
                proto.state.append(0)
        self.publish_dccl(proto)
            # self.local_report_roslaunch_tx_flag = True
    
    ##report GPIO power
    def report_gpio_callback(self):
        # if(self.local_report_gpio_tx_flag == False):
        if self.local_report_gpio_client.service_is_ready():
            flag = self.local_report_gpio_client.wait_for_service(timeout_sec=self.ser_wait_time)
            if flag:
                request =Trigger.Request()
                future = self.local_report_gpio_client.call_async(request)
                future.add_done_callback(self.report_gpio_state_callback_done)
            # data = []
            else:
                print(f'Service: [{self.local_report_gpio_client.srv_name}] Timeout', flush=True)
        else:
            print(f'Service: [{self.local_report_gpio_client.srv_name}] Not available', flush=True)

    def report_gpio_state_callback_done(self, future):
        response = future.result()
        proto = mvp_cmd_dccl_pb2.ReportPowerPort()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id

        power_list = response.message.splitlines()
        for count, line in enumerate(power_list):
            parts = line.split('=')
            if len(parts) == 2:  # Ensure there are exactly two parts
                name, status = parts
                proto.state.append(int(status))
                
        self.publish_dccl(proto)
        # self.local_report_gpio_tx_flag = True

    #report controller backback
    def report_controller_state_callback(self):
        # Block until the service is available (1-second timeout)
        # try:
        if self.local_report_controller_client.service_is_ready():
            flag = self.local_report_controller_client.wait_for_service(timeout_sec=self.ser_wait_time)
            if flag:
                # self.get_logger().info(f"Waiting for service '{self.local_report_controller_client.srv_name}' to become available...")
                request = Trigger.Request()
                future = self.local_report_controller_client.call_async(request)

                # Handle the response
                future.add_done_callback(self.report_controller_state_callback_done)
            else:
                print(f'Service: [{self.local_report_controller_client.srv_name}] Timeout', flush=True)
        else: 
                print(f'Service: [{self.local_report_controller_client.srv_name}] Not available', flush=True)


    def report_controller_state_callback_done(self, future):
        response = future.result()
        # self.get_logger().info(f'Service response: {response.message}')
        ##make dccl
        proto = mvp_cmd_dccl_pb2.ReportController()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.status = response.message == "enabled"
        # print(proto, flush = True)
        # if self.local_report_controller_state_tx_flag is False:
        self.publish_dccl(proto)
            # self.local_report_controller_state_tx_flag = True
        return response      


    def report_helm_state_callback(self):
        # Block until the service is available (1-second timeout)
        # try:
        if self.local_report_helm_client.service_is_ready():

            flag = self.local_report_helm_client.wait_for_service(timeout_sec=self.ser_wait_time)
            # self.get_logger().info(f"Waiting for service '{self.local_report_helm_client.srv_name}' to become available...")
            if flag:
                request = GetState.Request()
                request.name = ''
                future = self.local_report_helm_client.call_async(request)

                # Handle the response
                future.add_done_callback(self.report_helm_state_callback_done)
            else:
                print(f'Service: [{self.local_report_helm_client.srv_name}] Timeout', flush=True)
        else:
                print(f'Service: [{self.local_report_helm_client.srv_name}] Not available', flush=True)


    def report_helm_state_callback_done(self, future):
        response = future.result()
        # self.get_logger().info(f'Service response: {response}')
        ##make dccl
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
        # if self.local_report_helm_state_tx_flag is False:
        self.publish_dccl(proto)
            # print(proto, flush = True)
            # self.local_report_helm_state_tx_flag = True
        return response   
    
    def report_wpt_callback(self):
        if self.local_report_wpt_client.service_is_ready():

            flag = self.local_report_wpt_client.wait_for_service(timeout_sec=self.ser_wait_time)
            # self.get_logger().info(f"Waiting for service '{self.local_report_wpt_client.srv_name}' to become available...")
            if flag:
                request = GetWaypoints.Request()
                request.count.data = 0
                future = self.local_report_wpt_client.call_async(request)

                # Handle the response
                future.add_done_callback(self.report_wpt_callback_done)
            else:
                # print("Get Waypoint Service Timeout")
                print(f'Service: [{self.local_report_wpt_client.srv_name}] Timeout', flush=True)
        else:
                print(f'Service: [{self.local_report_wpt_client.srv_name}] not available', flush=True)
            
    def report_wpt_callback_done(self, future):
        response = future.result()
        # print(response)
        # self.get_logger().info(f'Service response: {response}')
        ##make dccl
        proto = mvp_cmd_dccl_pb2.ReportWpt()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.wpt_size = len(response.wpt)
        # print(proto.wpt_size, flush=True)
        for i in range(proto.wpt_size):
            proto.latitude.append(response.wpt[i].ll_wpt.latitude*100)
            proto.longitude.append(response.wpt[i].ll_wpt.longitude*100)
            proto.altitude.append(response.wpt[i].ll_wpt.altitude)    
            proto.u.append(response.wpt[i].u)
            # print (i)                           

        # if self.local_report_wpt_tx_flag is False:
            # dccl_msg = ByteMultiArray()
            # dccl_msg.data = self.dccl_obj.encode(proto)
            # dccl_msg.data = package_dccl(dccl_msg.data)
            # print(len(dccl_msg.data))
            # print(dccl_msg.data, flush=True)
        self.publish_dccl(proto)
            # print(proto, flush = True)
            # self.local_report_wpt_tx_flag = True
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