def package_dccl(data):
    # Calculate the checksum for the NMEA string
    checksum = 0
    for byte in data:
        checksum ^= byte     
    ender_bytes = f"*{checksum:02X}"
    data_out = bytearray('$$$', 'ascii') + data +  bytearray(ender_bytes+'\n', 'ascii')
    return data_out


def check_dccl(data):
        ##check the header
        if data[:3] != bytearray([36, 36, 36]): 
            print("Error: Header imcomplete")
            return False
        ##check the * char
        if data[-4] != 42:
            print("Error: Data does not end with '*'")
            return False
        #get checksum string
        checksum_str = bytes([data[-3], data[-2]]).decode('ascii')
        #compute checksum
        calculated_checksum = 0
        for byte in data[3:-4]: 
            calculated_checksum ^= byte
        # Format the checksum 
        calculated_checksum_str = f"{calculated_checksum:02X}"
        
        #compare checksum with the calculated checksum
        if calculated_checksum_str == checksum_str:
            # print("Data is complete and valid.")
            data_extracted = data[3:-4]
            data_out = bytes(data_extracted)
            return True, data_out
        else:
            print("Error: Checksum does not match")
            return False