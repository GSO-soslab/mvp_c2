import socket
import time

class UDPInterface:
    def __init__(self, type, udp_srv_ip, udp_srv_port, udp_client_ip, udp_client_port):
        self.type = type
        self.max_retries = 10
        self.server_ip = udp_srv_ip
        self.server_port = udp_srv_port
        self.server_addr = (self.server_ip, self.server_port)

        self.client_ip = udp_client_ip
        self.client_port = udp_client_port
        self.client_addr = (udp_client_ip, udp_client_port)
        
        if self.type == 'server':
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.server_socket.bind(self.server_addr)
            print(f"UDP server is up on {self.server_ip}:{self.server_port}", flush=True)
            print(f"waiting for {self.client_ip}: {self.client_port}", flush=True)


        if self.type == 'client':
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.client_socket.bind(self.client_addr)
            print(f"UDP client is setup on {self.client_ip}:{self.client_port}", flush=True)
            print(f"will connect to server: {self.server_ip}:{self.server_port}", flush=True)

    def send(self, data):  
        if self.type == 'server':
             self.server_socket.sendto(data, self.client_addr)
            
        elif self.type == 'client':
            self.client_socket.sendto(data, self.server_addr)


    def read(self):
        try:
            if self.type == 'server':
                # self.server_socket.settimeout(1)  # Set timeout to  1 seconds
                data, addr = self.server_socket.recvfrom(1024)  # Buffer size is 1024 bytes
                # print(addr, flush = True)
                # print(data, flush = True)
                if addr[0] == self.client_addr[0]:
                    return data
                else:
                    print("No UDP data", flush =True)
                    return None
            
            elif self.type == 'client':
                # self.client_socket.settimeout(1)  # Set timeout to  1 seconds
                data, addr = self.client_socket.recvfrom(1024)  # Buffer size is 1024 bytes
                # print(addr, flush = True)
                # print(data, flush = True)
                if addr[0] == self.server_addr[0]:
                    return data
                else:
                    print("No UDP data", flush =True)
                    return None
        except socket.timeout:
            print("Socket timed out!", flush = True)
            return None  # Return None or handle timeout case as needed
        

    def close(self):
        print("closing the socket", flush = True)
        # if(self.connection_status):
        if self.type == 'server':
            self.server_socket.close()
            
        elif self.type == 'client':
            self.client_socket.close()