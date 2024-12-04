import rclpy
from rclpy.node import Node
import threading

from serial_interface import UDPInterface
from std_msgs.msg import UInt8MultiArray


class MvpC2SerialRos(Node):
    def __init__(self):
        super().__init__('mvp_c2_udp')
        #parameters
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baud = self.declare_parameter('baudrate', 115200).value
        self.rx_timer = self.declare_parameter('rx_timer', 0.1).value

        self.ser = SerialInterface(self.port, self.baud)

        ##subscribe to dccl tx topic
        self.dccl_tx_sub = self.create_subscription(UInt8MultiArray, 'dccl_msg_tx', self.dccl_tx_callback, 10)

        ##publish to dccl rx topic
        self.ddcl_rx_pub = self.create_publisher(UInt8MultiArray, 'dccl_msg_rx', 10)
        
        self.running = True
        threading.Thread(target=self.dccl_rx_callback, daemon=True).start()
        print("Serial listener started.")

    def dccl_tx_callback(self, msg):
        # print("got dccl", flush =True)
        # print(msg.data, flush = True)
        self.ser.send(msg.data)

    def dccl_rx_callback(self):
        while self.running:
            try:
                data = self.ser.read()
                if(data is not False):
                    msg = UInt8MultiArray()
                    msg.data = data
                    # print("publishing", flush = True)
                    self.ddcl_rx_pub.publish(msg)
            except Exception as e:
                print(f"Error in dccl_rx_callback: {e}")
                break

    def close_udp(self):
        self.running = False
        self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = MvpC2SerialRos()
    rclpy.spin(node)  # Keep the node running to allow timer execution
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
