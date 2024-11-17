import rclpy
from rclpy.node import Node
import threading

from udp_interface import UDPInterface
from std_msgs.msg import UInt8MultiArray


class MvpC2UdpRos(Node):
    def __init__(self):
        super().__init__('mvp_c2_udp')
        #parameters
        self.udp_type = self.declare_parameter('type', 'server').value
        self.server_ip = self.declare_parameter('server_ip', '192.168.0.123').value
        self.server_port = self.declare_parameter('server_port', 2000).value
        self.client_ip = self.declare_parameter('client_ip', '192.168.0.100').value
        self.client_port = self.declare_parameter('client_port', 3000).value
        self.rx_timer = self.declare_parameter('rx_timer', 0.1).value

        self.udp_obj = UDPInterface(self.udp_type, self.server_ip, self.server_port, 
                                    self.client_ip, self.client_port)

        ##subscribe to dccl tx topic
        self.dccl_tx_sub = self.create_subscription(UInt8MultiArray, 'dccl_msg_tx', self.dccl_tx_callback, 10)

        ##publish to dccl rx topic
        self.ddcl_rx_pub = self.create_publisher(UInt8MultiArray, 'dccl_msg_rx', 10)
        
        # self.timer = self.create_timer(self.rx_timer, self.dccl_rx_callback)
        self.timer_test = self.create_timer(2.0, self.udp_test)

        self.running = True
        threading.Thread(target=self.dccl_rx_callback, daemon=True).start()
        print("UDP listener started.")

    def udp_test(self):
        data = f"I am {self.udp_type}"
        self.udp_obj.send(data.encode('utf-8'))
        

    def dccl_tx_callback(self, msg):
        print("got dccl", flush =True)
        print(msg.data, flush = True)
        self.udp_obj.send(msg.data)

    def dccl_rx_callback(self):
        while self.running:
            try:
                data = self.udp_obj.read()
                if(data is not None):
                    self.ddcl_rx_pub.publish(data)
            except Exception as e:
                print(f"Error in dccl_rx_callback: {e}")
                break

    def close_udp(self):
        self.running = False
        self.udp_obj.close()

def main(args=None):
    rclpy.init(args=args)
    node = MvpC2UdpRos()
    rclpy.spin(node)  # Keep the node running to allow timer execution
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
