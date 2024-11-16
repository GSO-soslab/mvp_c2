import socket


class UDPInterface:
    def __init__(self, type, udp_srv_ip, udp_srv_port, udp_client_ip, udp_client_port):
        self.type = type
       
        self.server_addr = (udp_srv_ip, udp_srv_port)
        self.client_addr = (udp_client_ip, udp_client_port)

        if type == 'server':
            self.srv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.srv_socket.bind(self.server_addr)  # Bind to localhost on port 65432
            self.srv_socket.settimeout(10000.0)

        elif type == 'client':
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.client_socket.bind(self.client_addr)  # Bind to localhost on port 65432
            self.client_socket.settimeout(10000.0)

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