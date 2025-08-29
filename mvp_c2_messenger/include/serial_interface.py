import serial

class SerialInterface:
    def __init__(self, port_name, baud_rate):
        self.ser = serial.Serial(port_name, baud_rate, timeout=1)  # Replace with your port and baud rate
        self.ser.flush()

    def send(self, data):
        # message = data
        self.ser.write(data)
    
    
    def send_packet(self, data):
        max_bytes = 20
        # message = data
        # encoded_message = message.encode('utf-8') 
        # Send data in chunks
        for i in range(0, len(data), max_bytes):
            chunk = data[i:i + max_bytes]
            self.ser.write(chunk)
            self.ser.flush()  # Ensure data is sent immediately

    def read(self):
        if self.ser.in_waiting > 0:
            data = self.ser.readline()
            return data
        else:
            return False


    def close(self):
        self.ser.close()    