START = b'$$$'


def package_dccl(data):
    # Calculate the checksum for the NMEA string
    checksum = 0
    for byte in data:
        checksum ^= byte     
    ender_bytes = f"*{checksum:02X}"
    data_out = bytearray('$$$', 'ascii') + data +  bytearray(ender_bytes+'\n', 'ascii')
    return data_out

def calc_checksum(payload: bytes) -> int:
    cs = 0
    for b in payload:
        cs ^= b
    return cs & 0xFF

def extract_packets(buffer: bytearray):
    """Yield (payload_bytes, full_packet_bytes) for each valid packet; mutate buffer to remove consumed bytes."""
    out = []
    i = 0
    while True:
        # Find start
        s = buffer.find(START, i)
        if s == -1:
            # no start marker; drop everything before current end
            del buffer[:]
            break

        # Drop everything before the start
        if s > 0:
            del buffer[:s]
            s = 0

        # We have START at position 0. Look for '*' then 2 hex digits and '\n'
        star = buffer.find(b'*', len(START))
        if star == -1:
            # need more data
            break

        # Need at least '*' + 2 hex + '\n'
        if star + 3 >= len(buffer):
            # not enough yet
            break

        # Check that we have two hex digits and a newline following
        if star + 3 < len(buffer) and buffer[star+3-0:star+4] != b'\n':
            # We don't know yet if '\n' is present; if not present yet, wait for more data
            if buffer[star+3] != 0x0A:  # '\n'
                if len(buffer) < star + 4:
                    break  # wait for more bytes
        # Now ensure we actually have "*XX\n"
        if len(buffer) < star + 4:
            break
        hex2 = buffer[star+1:star+3]
        nl = buffer[star+3]
        if nl != 0x0A or any(c not in b'0123456789ABCDEFabcdef' for c in hex2):
            # Malformed sequence after '*': skip this '*' and continue search
            i = star + 1
            continue

        # Candidate full packet is: START + payload + '*' + XX + '\n'
        payload = bytes(buffer[len(START):star])
        recv_cs = int(hex2.decode('ascii'), 16)
        calc_cs = calc_checksum(payload)

        full_len = star + 4  # inclusive of newline
        full_packet = bytes(buffer[:full_len])

        if recv_cs == calc_cs:
            out.append((payload, full_packet))
            # Consume this packet from the buffer
            del buffer[:full_len]
            i = 0  # restart scanning buffer from beginning
        else:
            # Bad checksum: drop the leading '$' and continue (resync)
            del buffer[0:1]
            i = 0

    return out


def check_dccl(data):
        flag = False
        data_out = data  #
        #check the header
        if data[:3] != bytearray([36, 36, 36]): 
            print("Error: Header imcomplete", flush = True)
            # print(data, flush = True)
            # print(f'data size = {len(data)}')
            return flag, data_out
        ##check the * char
        elif data[-4] != 42:
            print("Error: Data does not end with '*'", flush = True)
            return flag, data_out
        #get checksum string
        elif(len(data)<7):
             print ("Data is not long enough")
             return flag, data_out
        else:
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
                flag = True
            else:
                print("Error: Checksum does not match")
        return flag, data_out