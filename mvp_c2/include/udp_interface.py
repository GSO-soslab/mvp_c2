import socket
import time

class UDPInterface:
    def __init__(self, type, udp_srv_ip, udp_srv_port, udp_client_ip, udp_client_port):
        self.type = type
        self.max_retries = 10
        self.server_addr = (udp_srv_ip, udp_srv_port)
        self.server_ip = udp_srv_ip
        self.server_port = udp_srv_port
        self.client_addr = (udp_client_ip, udp_client_port)
        self.client_ip = udp_client_ip
        self.client_port = udp_client_port

        self.connection_status = False
        
        self.connect()

    def connect(self):
        self.connection_status = False
        
        #loop until connection is good

        while(self.connection_status == False):
            try:
                if self.type == 'server':
                    print("Checking connection", flush = True)
                    self.srv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    self.srv_socket.bind(self.server_addr)
                    self.srv_socket.settimeout(1.0)
                    self.srv_socket.connect(self.client_addr)
                    #send a ping
                    print("Pinging", flush = True)
                    self.srv_socket.send(b"Ping")
                    self.srv_socket.settimeout(1.0)  # Wait for up to 5 seconds
                    response, addr = self.srv_socket.recvfrom(1024)
                    print("Client is reachable:", addr)
                    self.connection_status = True
                    # print(f"Server: IP {self.client_ip} on port {self.client_port} is reachable.")

                elif self.type == 'client':
                    self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    self.client_socket.bind(self.client_addr)
                    self.client_socket.settimeout(1.0)
                    # Try connecting to the server
                    self.client_socket.connect(self.server_addr)
                    print("Pinging", flush = True)

                    # print(f"Client: IP {self.server_ip} on port {self.server_port} is reachable.")
                    #send a ping
                    self.srv_socket.send(b"Ping")
                    self.srv_socket.settimeout(1)  # Wait for up to 5 seconds
                    response, addr = self.srv_socket.recvfrom(1024)
                    print("Server is reachable:", addr)
                    self.connection_status = True
            except socket.timeout:
                print("Connection Timeout", flush = True)
                self.connection_status = False  # Set to False if there's an error
                
            except socket.error as e:
                # Handle other socket errors (e.g., connection refused or unreachable)
                print(f"Socket error: {e}")
            time.sleep(2.0)

    def send(self, data):  
        if self.type == 'server':
             self.srv_socket.sendto(data, self.client_addr)
            
        elif self.type == 'client':
            self.client_socket.sendto(data, self.server_addr)


    def read(self):
        self.srv_socket.settimeout(1)  # Set timeout to 5 seconds
        try:
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
        except socket.timeout:
            print("Socket timed out!", flush = True)
            return False  # Return None or handle timeout case as needed
        

    def close(self):
        print("closing the socket", flush = True)
        if(self.connection_status):
            if self.type == 'server':
                self.srv_socket.close()
                
            elif self.type == 'client':
                self.client_socket.close()