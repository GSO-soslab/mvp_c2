import rclpy
from rclpy.node import Node
import threading

from include.serial_interface import SerialInterface
from std_msgs.msg import ByteMultiArray, UInt8MultiArray


class MvpC2SerialRos(Node):
    def __init__(self):
        super().__init__('mvp_c2_serial_comm')
        #parameters
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baud = self.declare_parameter('baudrate', 9600).value
        self.rx_timer = self.declare_parameter('rx_timer', 0.1).value

        
        self.ser = SerialInterface(self.port, self.baud)
        # print("port open", flush=True)
        print(f"port: {self.port}, baud: {self.baud} is open", flush=True)

        ##subscribe to dccl tx topic
        self.dccl_tx_sub = self.create_subscription(ByteMultiArray, 'dccl_msg_tx', self.dccl_tx_callback, 10)

        ##publish to dccl rx topic
        self.ddcl_rx_pub = self.create_publisher(ByteMultiArray, 'dccl_msg_rx', 10)
        
        self.running = True
        threading.Thread(target=self.dccl_rx_callback, daemon=True).start()
        print("Serial started")

    def dccl_tx_callback(self, msg):
        data = bytearray(ord(c) for c in msg.data)  # if msg.data is a list of characters
        # print(f'send:{len(data)}', flush=True)
        self.ser.send(data)

    def dccl_rx_callback(self):
        while self.running:
            data = bytearray([])
            counter = 1
            while True:
                temp_data = self.ser.read()
                if temp_data is not False:
                    # print(temp_data)
                    data = data + temp_data
                    counter = counter + 1

                    if len(data) >= 3 and data[-4] == 42: #the four last chars are *AB\n
                        msg = ByteMultiArray()
                        msg.data = data
                        # print(msg.data)
                        # print(f'received:{len(msg.data)}', flush=True)
                        self.ddcl_rx_pub.publish(msg)
                        break 
                    if counter == 5:
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
