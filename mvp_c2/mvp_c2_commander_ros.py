import rclpy
import sys, os
import dccl
import signal
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, ByteMultiArray, Int16MultiArray, Float32MultiArray
from sensor_msgs.msg import Joy
from geographic_msgs.msg import GeoPath
from mvp_msgs.srv import SetString, SendWaypoints
from mvp_msgs.msg import HelmState, Waypoints, Waypoint
from geometry_msgs.msg import PointStamped


from std_srvs.srv import Trigger, SetBool
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
# from mvp_msgs.srv import GetControlMode

from include.dccl_checksum import check_dccl, package_dccl

# sys.path.append('../proto')  # Adjust path if needed
import mvp_cmd_dccl_pb2
import time 
from ament_index_python.packages import get_package_share_directory

package_name = 'mvp_c2'

# Full path to the config file



class MvpC2Commander(Node):

    def __init__(self):
        super().__init__('mvp_c2_commander_ros')

        # ns = self.get_namespace()

        self.local_id = self.declare_parameter('local_id', 1).value
        self.remote_id = self.declare_parameter('remote_id', 2).value
        self.dccl_rx_interval = self.declare_parameter('dccl_rx_interval', 1.0).value

        self.dccl_tx_joy_interval = self.declare_parameter('dccl_tx_joy_interval', 0.1).value

        # self.default_state_list = ['start', 'kill', 'survey', 'profiling', 'teleop']
        self.declare_parameter('helm_state_list', [''])
        self.default_state_list = self.get_parameter('helm_state_list').get_parameter_value().string_array_value
        
        self.declare_parameter('launch_packages', [''])
        self.launch_packages = self.get_parameter('launch_packages').get_parameter_value().string_array_value
     
        self.declare_parameter('launch_files', [''])
        self.launch_file_names = self.get_parameter('launch_files').get_parameter_value().string_array_value

        print("load gpio device", flush = True)
        self.declare_parameter('gpio_devices', [''])
        self.gpio_devices = self.get_parameter('gpio_devices').get_parameter_value().string_array_value

        # print(self.gpio_devices, flush=True)

        # mvp_active meaning the local machine has mvp running so it can transfer its mvp related 
        ##publish information parsed from dccl to ros topic
        topic_prefix = 'mvp_c2_commander/remote/id_' + str(self.remote_id)
        self.remote_odom_pub = self.create_publisher(Odometry, topic_prefix + '/odometry', 10)
        self.remote_geopose_pub = self.create_publisher(GeoPoseStamped, topic_prefix + '/geopose', 10)
        self.remote_acomm_navsat_pub = self.create_publisher(NavSatFix, topic_prefix + '/acomm/navsatfix', 10)

        self.remote_odom_navsat_pub = self.create_publisher(NavSatFix, topic_prefix + '/odometry/navsatfix', 10)
        self.remote_controller_state_pub = self.create_publisher(Bool, topic_prefix + '/controller_state', 10)
        self.remote_helm_state_pub = self.create_publisher(HelmState, topic_prefix + '/helm/state', 10)
        self.remote_wpt_report_pub = self.create_publisher(Waypoints, topic_prefix + '/survey/geopath', 10)
        self.remote_roslaunch_report_pub = self.create_publisher(Int16MultiArray, topic_prefix +'/roslaunch_state', 10)
        self.remote_gpio_power_report_pub = self.create_publisher(Int16MultiArray, topic_prefix+'/gpio_power_state',10)
        self.remote_power_info_pub = self.create_publisher(Float32MultiArray, topic_prefix + '/power_info',10)
        self.remote_cpu_info_pub = self.create_publisher(Float32MultiArray, topic_prefix + '/computer_info',10)
        self.remote_altimeter_pub = self.create_publisher(PointStamped, topic_prefix + '/altimeter',10)
        self.remote_feature_points_pub = self.create_publisher(Float32MultiArray, topic_prefix + '/feature_points',10)


        self.local_joy_sub = self.create_subscription(Joy, topic_prefix + '/joy', self.joy_callback, 10)

        ##service for access remote controllers
        self.remote_set_controller_srv = self.create_service(SetBool, topic_prefix + '/controller/set', self.remote_set_controller_callback)
        self.remote_set_state_srv = self.create_service(SetString, topic_prefix + '/mvp_helm/change_state', self.remote_set_helm_state_callback)
        self.remote_set_wpt_srv = self.create_service(SendWaypoints, topic_prefix + '/mvp_helm/set_waypoints', self.remote_set_waypoints_callback)
        self.remote_reset_datum_srv = self.create_service(Trigger, topic_prefix + '/reset_datum', self.reset_datum_callback)

        ##service for roslaunch files
        if len(self.launch_packages) == len(self.launch_file_names):
            for index in range(len(self.launch_packages)):
                srv_name = topic_prefix + '/roslaunch/' + self.launch_packages[index] + '/' + self.launch_file_names[index]
                self.remote_set_state_srv = self.create_service(
                    SetBool,
                    srv_name,
                    lambda req, resp, idx=index: self.roslaunch_srv_callback(req, resp, idx)
                )

        else:
            print("Launch packages and name entries doesn't match", flush = True)


        ##service for gpio
        # print(len(self.gpio_devices))
        # if len(self.gpio_devices) > 0:
        for index in range(len(self.gpio_devices)):
            srv_name = topic_prefix + '/gpio_manager/set_power/' + self.gpio_devices[index]
            self.remote_set_state_srv = self.create_service(
                SetBool,
                srv_name,
                lambda req, resp, idx=index: self.set_gpio_callback(req, resp, idx)
            )


        #DCCL byte array topic
        self.ddcl_reporter_pub = self.create_publisher(ByteMultiArray, 'mvp_c2/commander/dccl_msg_tx', 10)

        self.dccl_reporter_sub = self.create_subscription(ByteMultiArray, 
                                                        'mvp_c2/commander/dccl_msg_rx', 
                                                        self.dccl_rx_callback, 10)


        dccl.loadProtoFile(os.path.join( get_package_share_directory(package_name), 
                                         'proto', 
                                         'mvp_cmd_dccl.proto') )
        
        self.dccl_obj = dccl.Codec()
        self.load_dccl()
        print("mvp_c2 commander initialized", flush=True)

        ##timer for resetting the dccl tx flag
        # self.local_joy_tx_flag = False
        # self.remote_set_controller_tx_flag = False
        # self.remote_set_helm_state_tx_flag = False
        # self.remote_set_ros_launch_tx_flag = False
        # self.remote_set_wpt_tx_flag = False
        # self.remote_set_power_tx_flag = False
        # self.remote_reset_datum_flag = False

    #     self.timer = self.create_timer(self.dccl_tx_interval, self.reset_dccl_tx_flag)
    #     self.timer2 = self.create_timer(self.dccl_tx_joy_interval, self.reset_dccl_tx_joy_flag)


    # def reset_dccl_tx_flag(self):
    #     self.remote_set_controller_tx_flag = False
    #     self.remote_set_helm_state_tx_flag = False
    #     self.remote_set_ros_launch_tx_flag = False
    #     self.remote_set_wpt_tx_flag = False
    #     self.remote_set_power_tx_flag = False
    #     self.remote_reset_datum_flag = False


    # def reset_dccl_tx_joy_flag(self):
    #     self.local_joy_tx_flag = False

    def load_dccl(self):
        self.dccl_obj.load('Joy')
        self.dccl_obj.load('PWM')
        self.dccl_obj.load('Odometry')
        self.dccl_obj.load('GeoPose')
        self.dccl_obj.load('PowerMonitor')
        self.dccl_obj.load('CPUMonitor')
        self.dccl_obj.load('AltimeterPointStamped')
        self.dccl_obj.load('AcommGeoPoint')
        self.dccl_obj.load('FeatureGeoPoints')
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
        self.dccl_obj.load('TdmaMasterSyncMsg')
        
        ##make a dictionary to keep the time of the last time a dccl msg came in
        # self.last_dccl_rx_time = {}  # {msg_id: last_received_time}
        # for i in range (50):
        #     self.last_dccl_rx_time[i] = time.time()
        self.last_dccl_rx_time = [time.time()] * 50

    #######################################################
    ############DCCL parsing###############################
    #######################################################
    ###parsing dccl
    def dccl_rx_callback(self,msg):
        # flag,data = check_dccl(msg.data)
        bdata = bytearray(ord(c) for c in msg.data)
        flag, data = check_dccl(bdata)
        if flag == True:
            message_id = self.dccl_obj.id(data)
            # print(message_id, flush = True)
            print(f'{round(time.time(), 3)}: dccl_message_id: {message_id}, data_len: {len(data)}', flush=True)

            proto_msg = self.dccl_obj.decode(data)
            # if proto_msg.remote_id == self.local_id:
            if"remote_id" in proto_msg.DESCRIPTOR.fields_by_name  and proto_msg.remote_id == self.local_id:
                #odometry 
                #checking msg_id time
                if time.time() - self.last_dccl_rx_time[message_id] < self.dccl_rx_interval:
                    print(f"The same message id={message_id} recived, deemed redudant", flush=True)
                    return
                else:
                    #update the time and proceed to decoding
                    self.last_dccl_rx_time[message_id] = time.time()

                if message_id == 3:
                    try:
                        # print(decoded_msg, flush = True)
                        msg = Odometry()
                        sec = int(proto_msg.time)  
                        nanosec = int((proto_msg.time - sec) * 1e9)  
                        msg.header.stamp.sec = sec
                        msg.header.stamp.nanosec = nanosec
                        # msg.header.frame_id = proto_msg.frame_id
                        # msg.child_frame_id = proto_msg.child_frame_id
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
                        # proto_msg = self.dccl_obj.decode(data)
                        # print(decoded_msg, flush = True)
                        msg = GeoPoseStamped()
                        sec = int(proto_msg.time)  
                        nanosec = int((proto_msg.time - sec) * 1e9)  
                        msg.header.stamp.sec = sec
                        msg.header.stamp.nanosec = nanosec
                        # msg.header.frame_id = proto_msg.frame_id
                        msg.pose.position.latitude = proto_msg.latitude*0.01
                        msg.pose.position.longitude = proto_msg.longitude*0.01
                        msg.pose.position.altitude = proto_msg.altitude
                        if len(proto_msg.orientation) ==4:
                            msg.pose.orientation.x = proto_msg.orientation[0]
                            msg.pose.orientation.y = proto_msg.orientation[1]
                            msg.pose.orientation.z = proto_msg.orientation[2]
                            msg.pose.orientation.w = proto_msg.orientation[3]
                        self.remote_geopose_pub.publish(msg)

                        ##get navsatfix
                        msg = NavSatFix()
                        sec = int(proto_msg.time)  
                        nanosec = int((proto_msg.time - sec) * 1e9)  
                        msg.header.stamp.sec = sec
                        msg.header.stamp.nanosec = nanosec
                        msg.status.status = 0
                        msg.status.service = 1
                        msg.latitude =  proto_msg.latitude*0.01
                        msg.longitude = proto_msg.longitude*0.01
                        msg.altitude = proto_msg.altitude
                        msg.position_covariance_type = 1
                        self.remote_odom_navsat_pub.publish(msg)

                    except Exception as e:
                        # Print the exception message for debugging
                        print(f"Decoding error: {e}", flush=True)

                ##Power info message
                if message_id == 5:
                    try:
                        # proto_msg = self.dccl_obj.decode(data)
                        #call the service and make the dccl msg
                        msg = Float32MultiArray()
                        msg.data = [proto_msg.data[0], proto_msg.data[1]]
                        self.remote_power_info_pub.publish(msg)
                    except Exception as e:
                        # Print the exception message for debugging
                        print(f"Decoding error: {e}", flush=True)

                ##CPU info message
                if message_id == 6:
                    try:
                        # proto_msg = self.dccl_obj.decode(data)
                        #call the service and make the dccl msg
                        msg = Float32MultiArray()
                        msg.data = [proto_msg.data[0], proto_msg.data[1], proto_msg.data[2]]
                        self.remote_cpu_info_pub.publish(msg)
                    except Exception as e:
                        # Print the exception message for debugging
                        print(f"Decoding error: {e}", flush=True)

                ##ALtimeter pointstamped
                if message_id ==7:
                    try:
                        # proto_msg = self.dccl_obj.decode(data)
                        msg = PointStamped()
                        sec = int(proto_msg.time)  
                        nanosec = int((proto_msg.time - sec) * 1e9)  
                        msg.header.stamp.sec = sec
                        msg.header.stamp.nanosec = nanosec
                        msg.point.z = proto_msg.data
                        self.remote_altimeter_pub.publish(msg)
                    except Exception as e:
                        # Print the exception message for debugging
                        print(f"Decoding error: {e}", flush=True)


                ##acomm geopoint pointstamped
                if message_id ==8:
                    try:
                        # proto_msg = self.dccl_obj.decode(data)
                        msg = NavSatFix()
                        sec = int(proto_msg.time)  
                        nanosec = int((proto_msg.time - sec) * 1e9)  
                        msg.header.stamp.sec = sec
                        msg.header.stamp.nanosec = nanosec
                        msg.status.status = 0
                        msg.status.service = 1
                        msg.latitude =  proto_msg.latitude*0.01
                        msg.longitude = proto_msg.longitude*0.01
                        msg.altitude = proto_msg.altitude
                        msg.position_covariance_type = 1
                        self.remote_acomm_navsat_pub.publish(msg)

                    except Exception as e:
                        # Print the exception message for debugging
                        print(f"Decoding error: {e}", flush=True)
                #feature points message
                if message_id == 9:
                    try:
                        # proto_msg = self.dccl_obj.decode(data)
                        msg = Float32MultiArray()
                        sec = int(proto_msg.time)  
                        nanosec = int((proto_msg.time - sec) * 1e9)  
                        for i in range(proto_msg.point_size):
                            msg.data[i] = proto_msg.latitude[i]*0.01
                            msg.data[i+1] = proto_msg.longitude[i]*0.01
                            msg.data[i+2] = proto_msg.altitude[i]
                            self.remote_feature_points_pub.publish(msg)
                    except Exception as e:
                        # Print the exception message for debugging
                        print(f"Decoding error: {e}", flush=True)

                ##report controller message
                if message_id == 23:
                    try:
                        # proto_msg = self.dccl_obj.decode(data)
                        #call the service and make the dccl msg
                        msg = Bool()
                        msg.data = proto_msg.status
                        self.remote_controller_state_pub.publish(msg)
                    except Exception as e:
                        # Print the exception message for debugging
                        print(f"Decoding error: {e}", flush=True)

                ##report helm
                if message_id == 31:
                    try:
                        # proto_msg = self.dccl_obj.decode(data)
                        msg = HelmState()
                        msg.name = self.default_state_list[proto_msg.state]
                        msg.transitions = [self.default_state_list[i] for i in proto_msg.connected_state]
                        self.remote_helm_state_pub.publish(msg)
                    except Exception as e:
                        # Print the exception message for debugging
                        print(f"Decoding error: {e}", flush=True)

                ## report waypoints
                if message_id == 33:
                    # print("got Wpt", flush =True)
                    try:
                        # proto_msg = self.dccl_obj.decode(data)
                        msg = Waypoints()
                        sec = int(proto_msg.time)  
                        nanosec = int((proto_msg.time - sec) * 1e9)  
                        msg.wpt = [Waypoint() for _ in range(proto_msg.wpt_size)]
                        for i in range(proto_msg.wpt_size):
                            msg.wpt[i].header.stamp.sec = sec
                            msg.wpt[i].header.stamp.nanosec = nanosec
                            msg.wpt[i].header.frame_id = 'geopath'
                            msg.wpt[i].ll_wpt.latitude = proto_msg.latitude[i]*0.01
                            msg.wpt[i].ll_wpt.longitude = proto_msg.longitude[i]*0.01
                            msg.wpt[i].ll_wpt.altitude = proto_msg.altitude[i]
                            msg.wpt[i].u = proto_msg.u[i]
                        self.remote_wpt_report_pub.publish(msg)

                    except Exception as e:
                        # Print the exception message for debugging
                        print(f"Decoding error: {e}", flush=True)

                # report roslaunch status
                if message_id == 41:
                    try:
                        # proto_msg = self.dccl_obj.decode(data)
                        msg = Int16MultiArray()
                        msg.data = proto_msg.state
                        self.remote_roslaunch_report_pub.publish(msg)

                    except Exception as e:
                        # Print the exception message for debugging
                        print(f"Decoding error: {e}", flush=True)

                #report set power status
                if message_id == 21:
                    try:
                        # proto_msg = self.dccl_obj.decode(data)
                        msg = Int16MultiArray()
                        msg.data = proto_msg.state
                        self.remote_gpio_power_report_pub.publish(msg)
                    except Exception as e:
                        # Print the exception message for debugging
                        print(f"Decoding error: {e}", flush=True)
            else:
                print("DCCL filtered because remote_id don't match my local id", flush=True)
                

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

    #joy callback
    def joy_callback(self, msg):
        # print("got joy")
        proto = mvp_cmd_dccl_pb2.Joy()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.axes.extend(msg.axes)  # Map axes
        proto.buttons.extend(msg.buttons)  # Map buttons
        # if self.local_joy_tx_flag is False:
        self.publish_dccl(proto)
            # self.local_joy_tx_flag = True

    #set remote controller service callback
    def remote_set_controller_callback(self, request, response):
        # print("got set controller")
        proto = mvp_cmd_dccl_pb2.SetController()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.status = request.data
        response.success = True
        response.message = 'Service called'
        # if self.remote_set_controller_tx_flag is False:
        self.publish_dccl(proto)
            # self.remote_set_controller_tx_flag = True
        return response        
    
    ##set remote helm state
    def remote_set_helm_state_callback(self, request, response):
        proto = mvp_cmd_dccl_pb2.SetHelm()
        # proto.time = msg.header.stamp.to_sec()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        if request.data in self.default_state_list:
            index = self.default_state_list.index(request.data)
        else:
            print(f"'{request.data}' not found in default_state_list")
            index = 0
        proto.state = index
        # proto.state = request.data
        response.success = True
        response.message = 'Service called'
        # if self.remote_set_helm_state_tx_flag is False:
        self.publish_dccl(proto)
            # self.remote_set_helm_state_tx_flag = True
        return response        

    #set waypoints
    def remote_set_waypoints_callback(self, request, response):
        # print(request, flush =True)
        # print(len(request.wpt), flush =True)
        proto = mvp_cmd_dccl_pb2.SetWpt()
        proto.time = round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.wpt_size = len(request.wpt)
        if proto.wpt_size > 0:
            for i in range(proto.wpt_size):
                proto.latitude.append(request.wpt[i].ll_wpt.latitude*100)
                proto.longitude.append(request.wpt[i].ll_wpt.longitude*100)
                proto.altitude.append(request.wpt[i].ll_wpt.altitude) 
                proto.u.append(request.wpt[i].u)

            # if self.remote_set_wpt_tx_flag is False:
                # print(proto, flush=True)
            self.publish_dccl(proto)
                # self.remote_set_wpt_tx_flag = True

            response.success = True
            return response
        
            response.success = False
            return response
            
        response.success = False
        return response

    #reset datum
    def reset_datum_callback(self, request, response):
        proto = mvp_cmd_dccl_pb2.ResetDatum()
        proto.time = round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        # if self.remote_reset_datum_flag is False:
            # print(proto, flush=True)
        self.publish_dccl(proto)
            # self.remote_reset_datum_flag = True
            # response.success = True
        return response
        # response.success = False
        # return response

    #ros launch request callback
    def roslaunch_srv_callback(self, request, response, index):
        # Use some_int_value here
        proto = mvp_cmd_dccl_pb2.RosLaunch()
        proto.time =round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.index = index
        proto.req = request.data

        # print("service called")
        msg = f"{self.launch_packages[index]}/{self.launch_file_names[index]} | set to {request.data}"
        # Process request and prepare response
        response.success = True
        response.message = msg

        # if self.remote_set_ros_launch_tx_flag is False:
        self.publish_dccl(proto)
            # self.remote_set_ros_launch_tx_flag = True
        return response   

    def set_gpio_callback(self, request, response, index):
        proto = mvp_cmd_dccl_pb2.SetPowerPort()
        proto.time=round(time.time(), 3)
        proto.local_id = self.local_id
        proto.remote_id = self.remote_id
        proto.index = index
        proto.state = request.data
        msg = f"{self.gpio_devices[index]} Power set to {request.data}"
        # Process request and prepare response
        response.success = True
        response.message = msg

        # if self.remote_set_power_tx_flag is False:
        self.publish_dccl(proto)
        # self.remote_set_power_tx_flag = True
        return response   


    #make dccl

def main(args=None):
    rclpy.init(args=args)
    node = MvpC2Commander()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()