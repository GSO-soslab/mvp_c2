import sys
sys.path.append('../proto')  # Adjust path if needed
import os, dccl
import mvp_cmd_dccl_pb2
import time 

 
dccl.loadProtoFile(os.path.abspath("../proto/mvp_cmd_dccl.proto"))
 
codec = dccl.Codec()
codec.load("Joy")
 
# SENDER
joy_proto = mvp_cmd_dccl_pb2.Joy()

joy_proto.time = round(time.time(), 3)
joy_proto.source = 1
joy_proto.destination = 2
joy_proto.axes.extend([0.1, -0.2]) 
joy_proto.buttons.extend([1, 2, -1, 1, 1, 1, -1, 1, 1])

encoded_bytes = codec.encode(joy_proto)
print("encoded msg")
print(encoded_bytes)
# send encoded_bytes across your link
 
print("decoded msg")
# RECEIVER
decoded_msg = codec.decode(encoded_bytes)
print(decoded_msg)
