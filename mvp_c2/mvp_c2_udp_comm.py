import rclpy
from rclpy.node import Node
import threading

from udp_interface import UDPInterface
from std_msgs.msg import ByteMultiArray


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

        print (self.client_port, flush = True)
        self.udp_obj = UDPInterface(self.udp_type, self.server_ip, self.server_port, 
                                    self.client_ip, self.client_port)

        ##subscribe to dccl tx topic
        self.dccl_tx_sub = self.create_subscription(ByteMultiArray, 'dccl_msg_tx', self.dccl_tx_callback, 10)

        ##publish to dccl rx topic
        self.ddcl_rx_pub = self.create_publisher(ByteMultiArray, 'dccl_msg_rx', 10)
        
        self.running = True
        threading.Thread(target=self.dccl_rx_callback, daemon=True).start()
        print("UDP listener started.", flush = True)

    def dccl_tx_callback(self, msg):
        data = bytearray(ord(c) for c in msg.data)  # if msg.data is a list of characters
        # print("got dccl", flush =True)
        # print(msg.data, flush = True)
        self.udp_obj.send(data)

    def dccl_rx_callback(self):
        while self.running:
            data = bytearray([])
            try:
                data = self.udp_obj.read()
                if(data is not None):
                    msg = ByteMultiArray()
                    msg.data = data
                    # print("publishing", flush = True)
                    self.ddcl_rx_pub.publish(msg)
            except Exception as e:
                print(f"Error in dccl_rx_callback: {e}", flush = True)
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
