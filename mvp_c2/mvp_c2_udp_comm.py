import rclpy
from rclpy.node import Node
import threading
import time

from include.udp_interface import UDPInterface
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

        while True:
            try:
                print('Setting up UDP', flush= True)
                time.sleep(1)
                self.udp_obj = UDPInterface(self.udp_type, self.server_ip, self.server_port, 
                                            self.client_ip, self.client_port)
                
                break
            except Exception as e:
                print(f"Error in UDP setup: {e}", flush = True)

        print("UDP is good.", flush = True)
        ##subscribe to dccl tx topic
        self.dccl_tx_sub = self.create_subscription(ByteMultiArray, 'dccl_msg_tx', self.dccl_tx_callback, 10)

        ##publish to dccl rx topic
        self.ddcl_rx_pub = self.create_publisher(ByteMultiArray, 'dccl_msg_rx', 10)
        
        self.running = True
        threading.Thread(target=self.dccl_rx_callback, daemon=True).start()
            

           

    def dccl_tx_callback(self, msg):
        data = bytearray(ord(c) for c in msg.data)  # if msg.data is a list of characters
        try:
            self.udp_obj.send(data)
        except Exception as e:
            print(f"Error in dccl_tx_callback: {e}", flush = True)

    # def dccl_rx_callback(self):
    #     while self.running:
    #         data = bytearray([])
    #         try:
    #             data = self.udp_obj.read()
    #             if(data is not None):
    #                 msg = ByteMultiArray()
    #                 msg.data = data
    #                 # print("publishing", flush = True)
    #                 self.ddcl_rx_pub.publish(msg)
    #         except Exception as e:
    #             print(f"Error in dccl_rx_callback: {e}", flush = True)
    #             break
    def dccl_rx_callback(self):
        buffer = bytearray()
        while self.running:
            try:
                data = self.udp_obj.read()
                
                if not data:
                    continue
                print("data received", flush=True)  
                    
                buffer.extend(data)

                while True:
                    # Find start
                    start = buffer.find(b'$$')
                    if start == -1:
                        buffer.clear()
                        break

                    # Find newline AFTER start
                    end = buffer.find(b'\n', start)
                    if end == -1:
                        buffer = buffer[start:]
                        break

                    # Need at least "*XX\n" → 4 bytes
                    if end - start < 4:
                        buffer = buffer[start:]
                        break

                    # Exact terminator check
                    if buffer[end - 3] != ord('*'):
                        # Invalid frame → skip this '$$' and resync
                        buffer = buffer[start + 2:]
                        continue

                    # Extract full message
                    msg_bytes = buffer[start:end + 1]

                    # Remove consumed bytes
                    buffer = buffer[end + 1:]

                    msg = ByteMultiArray()
                    msg.data = msg_bytes
                    self.ddcl_rx_pub.publish(msg)

            except Exception as e:
                print(f"Error in dccl_rx_callback: {e}", flush=True)
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
