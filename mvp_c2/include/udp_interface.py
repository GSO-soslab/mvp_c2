import socket
import time

class UDPInterface:
    def __init__(self, type, udp_srv_ip, udp_srv_port, udp_client_ip, udp_client_port):
        self.type = type
        self.max_retries = 10
        self.server_addr = (udp_srv_ip, udp_srv_port)
        self.client_addr = (udp_client_ip, udp_client_port)
        try:
            if type == 'server':
                self.srv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.srv_socket.bind(self.server_addr)
                self.srv_socket.settimeout(5000.0)

                # Try connecting to the client (not strictly necessary for UDP, but for testing)
                self.srv_socket.connect(self.client_addr)
                print(f"Server: IP {udp_client_ip} on port {udp_client_port} is reachable.")
                self.connection_status = True  # Set to True if successful

            elif type == 'client':
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.client_socket.bind(self.client_addr)
                self.client_socket.settimeout(5000.0)

                # Try connecting to the server
                self.client_socket.connect(self.server_addr)
                print(f"Client: IP {udp_srv_ip} on port {udp_srv_port} is reachable.")
                self.connection_status = True  # Set to True if successful

        except (socket.timeout, socket.error) as e:
            # Notify the user but do not exit the program
            print(f"Error: Cannot reach {udp_client_ip if type == 'server' else udp_srv_ip} "
                  f"on port {udp_client_port if type == 'server' else udp_srv_port}. Error: {e}")
            print("The connection attempt failed. You may choose to retry or stop the program.")
            self.connection_status = False  # Set to False if there's an error


    def is_connected(self):
        return self.connection_status

    def send(self, data):
        
        if self.type == 'server':
             self.srv_socket.sendto(data, self.client_addr)
            
        elif self.type == 'client':
            self.client_socket.sendto(data, self.server_addr)


    def read(self):
        if self.type == 'server':
            data, addr = self.srv_socket.recvfrom(1024)  # Buffer size is 1024 bytes
            # print(addr)
            if addr[0] == self.client_addr[0]:
                return data
            else:
                return False
        
        elif self.type == 'client':
            data, addr = self.client_socket.recvfrom(1024)  # Buffer size is 1024 bytes
            if addr[0] == self.server_addr[0]:
                return data
            else:
                return False

    def close(self):

        if self.type == 'server':
            self.srv_socket.close()
            
        if self.type == 'client':
           self.client_socket.close()