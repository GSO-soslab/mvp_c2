import socket

def udp_server(host="192.168.0.123", port=12345):
    # Create a UDP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind((host, port))
    print(f"UDP server is listening on {host}:{port}")

    while True:
        # Wait for a client message
        data, client_address = server_socket.recvfrom(1024)
        print(f"Received message from {client_address}: {data.decode()}")

        # Send a response back to the client
        response = f"Echo: {data.decode()}"
        server_socket.sendto(response.encode(), client_address)

if __name__ == "__main__":
    udp_server()
