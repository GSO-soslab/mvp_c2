# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mvp_cmd_dccl.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from dccl import option_extensions_pb2 as dccl_dot_option__extensions__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x12mvp_cmd_dccl.proto\x1a\x1c\x64\x63\x63l/option_extensions.proto\"\xe3\x01\n\x03Joy\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12\'\n\x04\x61xes\x18\x04 \x03(\x01\x42\x19\xa2?\x16 \x02)\x00\x00\x00\x00\x00\x00\xf0\xbf\x31\x00\x00\x00\x00\x00\x00\xf0?P\x02\x12*\n\x07\x62uttons\x18\x05 \x03(\x05\x42\x19\xa2?\x16 \x00)\x00\x00\x00\x00\x00\x00\x00\xc0\x31\x00\x00\x00\x00\x00\x00\x00@P\t:\t\xa2?\x06\x08\x01\x10 (\x04\"\xdd\x01\n\x03PWM\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12&\n\x05index\x18\x04 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12%\n\x04\x64\x61ta\x18\x05 \x02(\x01\x42\x17\xa2?\x14 \x02)\x00\x00\x00\x00\x00\x00\xf0\xbf\x31\x00\x00\x00\x00\x00\x00\xf0?:\t\xa2?\x06\x08\x02\x10 (\x04\"\xf8\x02\n\x08Odometry\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12+\n\x08position\x18\x04 \x03(\x01\x42\x19\xa2?\x16 \x02)\x00\x00\x00\x00\x00\x88\xc3\xc0\x31\x00\x00\x00\x00\x00\x88\xc3@P\x03\x12.\n\x0borientation\x18\x05 \x03(\x01\x42\x19\xa2?\x16 \x02)\x00\x00\x00\x00\x00\x00\xf0\xbf\x31\x00\x00\x00\x00\x00\x00\xf0?P\x04\x12&\n\x03uvw\x18\x06 \x03(\x01\x42\x19\xa2?\x16 \x02)\x00\x00\x00\x00\x00\x00$\xc0\x31\x00\x00\x00\x00\x00\x00$@P\x03\x12&\n\x03pqr\x18\x07 \x03(\x01\x42\x19\xa2?\x16 \x02)\x00\x00\x00\x00\x00\x00$\xc0\x31\x00\x00\x00\x00\x00\x00$@P\x03\x12\x17\n\x08\x66rame_id\x18\x08 \x01(\tB\x05\xa2?\x02H\x0f\x12\x1d\n\x0e\x63hild_frame_id\x18\t \x01(\tB\x05\xa2?\x02H\x0f:\t\xa2?\x06\x08\x03\x10@(\x04\"\x83\x02\n\x07GeoPose\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12&\n\x03lla\x18\x04 \x03(\x01\x42\x19\xa2?\x16 \x02)\x00\x00\x00\x00\x00\x88\xc3\xc0\x31\x00\x00\x00\x00\x00\x88\xc3@P\x03\x12.\n\x0borientation\x18\x05 \x03(\x01\x42\x19\xa2?\x16 \x02)\x00\x00\x00\x00\x00\x00\xf0\xbf\x31\x00\x00\x00\x00\x00\x00\xf0?P\x04\x12\x17\n\x08\x66rame_id\x18\x08 \x01(\tB\x05\xa2?\x02H\x0f:\t\xa2?\x06\x08\x04\x10@(\x04\"\xe7\x01\n\x0cSetPowerPort\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12&\n\x05index\x18\x04 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12&\n\x05state\x18\x05 \x02(\x08\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00\x00@:\t\xa2?\x06\x08\x14\x10 (\x04\"\xc4\x01\n\x0fReportPowerPort\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12(\n\x05state\x18\x04 \x03(\x08\x42\x19\xa2?\x16 \x00)\x00\x00\x00\x00\x00\x00\xf0\xbf\x31\x00\x00\x00\x00\x00\x00\xf0?P\n:\t\xa2?\x06\x08\x15\x10 (\x04\"\xc1\x01\n\rSetController\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12\'\n\x06status\x18\x04 \x02(\x08\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\xf0\xbf\x31\x00\x00\x00\x00\x00\x00\xf0?:\t\xa2?\x06\x08\x16\x10 (\x04\"\xc4\x01\n\x10ReportController\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12\'\n\x06status\x18\x04 \x02(\x08\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\xf0\xbf\x31\x00\x00\x00\x00\x00\x00\xf0?:\t\xa2?\x06\x08\x17\x10 (\x04\"\xa8\x01\n\x07SetHelm\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12\x14\n\x05state\x18\x04 \x02(\tB\x05\xa2?\x02H\t:\t\xa2?\x06\x08\x1e\x10 (\x04\"\xcb\x01\n\nReportHelm\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12\x14\n\x05state\x18\x04 \x02(\tB\x05\xa2?\x02H(\x12\x1e\n\x0f\x63onnected_state\x18\x05 \x02(\tB\x05\xa2?\x02H(:\t\xa2?\x06\x08\x1f\x10\x64(\x04\"\xe5\x01\n\x06SetWpt\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12)\n\x08wpt_size\x18\x04 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12\'\n\x04\x64\x61ta\x18\x05 \x02(\x01\x42\x19\xa2?\x16 \x02)\x00\x00\x00\x00\x00\x88\xc3\xc0\x31\x00\x00\x00\x00\x00\x88\xc3@P\x1e:\t\xa2?\x06\x08 \x10 (\x04\"\xe8\x01\n\tReportWpt\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12)\n\x08wpt_size\x18\x04 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12\'\n\x04\x64\x61ta\x18\x05 \x02(\x01\x42\x19\xa2?\x16 \x02)\x00\x00\x00\x00\x00\x88\xc3\xc0\x31\x00\x00\x00\x00\x00\x88\xc3@P\x1e:\t\xa2?\x06\x08!\x10 (\x04\"\xe4\x01\n\tRosLaunch\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12&\n\x05index\x18\x04 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12&\n\x05state\x18\x05 \x02(\x08\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00\x00@:\t\xa2?\x06\x08(\x10 (\x04\"\xc4\x01\n\x0fReportRosLaunch\x12%\n\x04time\x18\x01 \x02(\x01\x42\x17\xa2?\x14 \x03)\x00\x00\x00\x00\x65\xcd\xcd\x41\x31\x00\x00\x00\x00\x65\xcd\xdd\x41\x12)\n\x08local_id\x18\x02 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12*\n\tremote_id\x18\x03 \x02(\x05\x42\x17\xa2?\x14 \x00)\x00\x00\x00\x00\x00\x00\x00\x00\x31\x00\x00\x00\x00\x00\x00Y@\x12(\n\x05state\x18\x04 \x03(\x08\x42\x19\xa2?\x16 \x00)\x00\x00\x00\x00\x00\x00\xf0\xbf\x31\x00\x00\x00\x00\x00\x00\xf0?P\n:\t\xa2?\x06\x08)\x10 (\x04')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'mvp_cmd_dccl_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _JOY.fields_by_name['time']._options = None
  _JOY.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _JOY.fields_by_name['local_id']._options = None
  _JOY.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _JOY.fields_by_name['remote_id']._options = None
  _JOY.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _JOY.fields_by_name['axes']._options = None
  _JOY.fields_by_name['axes']._serialized_options = b'\242?\026 \002)\000\000\000\000\000\000\360\2771\000\000\000\000\000\000\360?P\002'
  _JOY.fields_by_name['buttons']._options = None
  _JOY.fields_by_name['buttons']._serialized_options = b'\242?\026 \000)\000\000\000\000\000\000\000\3001\000\000\000\000\000\000\000@P\t'
  _JOY._options = None
  _JOY._serialized_options = b'\242?\006\010\001\020 (\004'
  _PWM.fields_by_name['time']._options = None
  _PWM.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _PWM.fields_by_name['local_id']._options = None
  _PWM.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _PWM.fields_by_name['remote_id']._options = None
  _PWM.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _PWM.fields_by_name['index']._options = None
  _PWM.fields_by_name['index']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _PWM.fields_by_name['data']._options = None
  _PWM.fields_by_name['data']._serialized_options = b'\242?\024 \002)\000\000\000\000\000\000\360\2771\000\000\000\000\000\000\360?'
  _PWM._options = None
  _PWM._serialized_options = b'\242?\006\010\002\020 (\004'
  _ODOMETRY.fields_by_name['time']._options = None
  _ODOMETRY.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _ODOMETRY.fields_by_name['local_id']._options = None
  _ODOMETRY.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _ODOMETRY.fields_by_name['remote_id']._options = None
  _ODOMETRY.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _ODOMETRY.fields_by_name['position']._options = None
  _ODOMETRY.fields_by_name['position']._serialized_options = b'\242?\026 \002)\000\000\000\000\000\210\303\3001\000\000\000\000\000\210\303@P\003'
  _ODOMETRY.fields_by_name['orientation']._options = None
  _ODOMETRY.fields_by_name['orientation']._serialized_options = b'\242?\026 \002)\000\000\000\000\000\000\360\2771\000\000\000\000\000\000\360?P\004'
  _ODOMETRY.fields_by_name['uvw']._options = None
  _ODOMETRY.fields_by_name['uvw']._serialized_options = b'\242?\026 \002)\000\000\000\000\000\000$\3001\000\000\000\000\000\000$@P\003'
  _ODOMETRY.fields_by_name['pqr']._options = None
  _ODOMETRY.fields_by_name['pqr']._serialized_options = b'\242?\026 \002)\000\000\000\000\000\000$\3001\000\000\000\000\000\000$@P\003'
  _ODOMETRY.fields_by_name['frame_id']._options = None
  _ODOMETRY.fields_by_name['frame_id']._serialized_options = b'\242?\002H\017'
  _ODOMETRY.fields_by_name['child_frame_id']._options = None
  _ODOMETRY.fields_by_name['child_frame_id']._serialized_options = b'\242?\002H\017'
  _ODOMETRY._options = None
  _ODOMETRY._serialized_options = b'\242?\006\010\003\020@(\004'
  _GEOPOSE.fields_by_name['time']._options = None
  _GEOPOSE.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _GEOPOSE.fields_by_name['local_id']._options = None
  _GEOPOSE.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _GEOPOSE.fields_by_name['remote_id']._options = None
  _GEOPOSE.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _GEOPOSE.fields_by_name['lla']._options = None
  _GEOPOSE.fields_by_name['lla']._serialized_options = b'\242?\026 \002)\000\000\000\000\000\210\303\3001\000\000\000\000\000\210\303@P\003'
  _GEOPOSE.fields_by_name['orientation']._options = None
  _GEOPOSE.fields_by_name['orientation']._serialized_options = b'\242?\026 \002)\000\000\000\000\000\000\360\2771\000\000\000\000\000\000\360?P\004'
  _GEOPOSE.fields_by_name['frame_id']._options = None
  _GEOPOSE.fields_by_name['frame_id']._serialized_options = b'\242?\002H\017'
  _GEOPOSE._options = None
  _GEOPOSE._serialized_options = b'\242?\006\010\004\020@(\004'
  _SETPOWERPORT.fields_by_name['time']._options = None
  _SETPOWERPORT.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _SETPOWERPORT.fields_by_name['local_id']._options = None
  _SETPOWERPORT.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _SETPOWERPORT.fields_by_name['remote_id']._options = None
  _SETPOWERPORT.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _SETPOWERPORT.fields_by_name['index']._options = None
  _SETPOWERPORT.fields_by_name['index']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _SETPOWERPORT.fields_by_name['state']._options = None
  _SETPOWERPORT.fields_by_name['state']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000\000@'
  _SETPOWERPORT._options = None
  _SETPOWERPORT._serialized_options = b'\242?\006\010\024\020 (\004'
  _REPORTPOWERPORT.fields_by_name['time']._options = None
  _REPORTPOWERPORT.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _REPORTPOWERPORT.fields_by_name['local_id']._options = None
  _REPORTPOWERPORT.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _REPORTPOWERPORT.fields_by_name['remote_id']._options = None
  _REPORTPOWERPORT.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _REPORTPOWERPORT.fields_by_name['state']._options = None
  _REPORTPOWERPORT.fields_by_name['state']._serialized_options = b'\242?\026 \000)\000\000\000\000\000\000\360\2771\000\000\000\000\000\000\360?P\n'
  _REPORTPOWERPORT._options = None
  _REPORTPOWERPORT._serialized_options = b'\242?\006\010\025\020 (\004'
  _SETCONTROLLER.fields_by_name['time']._options = None
  _SETCONTROLLER.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _SETCONTROLLER.fields_by_name['local_id']._options = None
  _SETCONTROLLER.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _SETCONTROLLER.fields_by_name['remote_id']._options = None
  _SETCONTROLLER.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _SETCONTROLLER.fields_by_name['status']._options = None
  _SETCONTROLLER.fields_by_name['status']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\360\2771\000\000\000\000\000\000\360?'
  _SETCONTROLLER._options = None
  _SETCONTROLLER._serialized_options = b'\242?\006\010\026\020 (\004'
  _REPORTCONTROLLER.fields_by_name['time']._options = None
  _REPORTCONTROLLER.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _REPORTCONTROLLER.fields_by_name['local_id']._options = None
  _REPORTCONTROLLER.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _REPORTCONTROLLER.fields_by_name['remote_id']._options = None
  _REPORTCONTROLLER.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _REPORTCONTROLLER.fields_by_name['status']._options = None
  _REPORTCONTROLLER.fields_by_name['status']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\360\2771\000\000\000\000\000\000\360?'
  _REPORTCONTROLLER._options = None
  _REPORTCONTROLLER._serialized_options = b'\242?\006\010\027\020 (\004'
  _SETHELM.fields_by_name['time']._options = None
  _SETHELM.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _SETHELM.fields_by_name['local_id']._options = None
  _SETHELM.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _SETHELM.fields_by_name['remote_id']._options = None
  _SETHELM.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _SETHELM.fields_by_name['state']._options = None
  _SETHELM.fields_by_name['state']._serialized_options = b'\242?\002H\t'
  _SETHELM._options = None
  _SETHELM._serialized_options = b'\242?\006\010\036\020 (\004'
  _REPORTHELM.fields_by_name['time']._options = None
  _REPORTHELM.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _REPORTHELM.fields_by_name['local_id']._options = None
  _REPORTHELM.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _REPORTHELM.fields_by_name['remote_id']._options = None
  _REPORTHELM.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _REPORTHELM.fields_by_name['state']._options = None
  _REPORTHELM.fields_by_name['state']._serialized_options = b'\242?\002H('
  _REPORTHELM.fields_by_name['connected_state']._options = None
  _REPORTHELM.fields_by_name['connected_state']._serialized_options = b'\242?\002H('
  _REPORTHELM._options = None
  _REPORTHELM._serialized_options = b'\242?\006\010\037\020d(\004'
  _SETWPT.fields_by_name['time']._options = None
  _SETWPT.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _SETWPT.fields_by_name['local_id']._options = None
  _SETWPT.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _SETWPT.fields_by_name['remote_id']._options = None
  _SETWPT.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _SETWPT.fields_by_name['wpt_size']._options = None
  _SETWPT.fields_by_name['wpt_size']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _SETWPT.fields_by_name['data']._options = None
  _SETWPT.fields_by_name['data']._serialized_options = b'\242?\026 \002)\000\000\000\000\000\210\303\3001\000\000\000\000\000\210\303@P\036'
  _SETWPT._options = None
  _SETWPT._serialized_options = b'\242?\006\010 \020 (\004'
  _REPORTWPT.fields_by_name['time']._options = None
  _REPORTWPT.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _REPORTWPT.fields_by_name['local_id']._options = None
  _REPORTWPT.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _REPORTWPT.fields_by_name['remote_id']._options = None
  _REPORTWPT.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _REPORTWPT.fields_by_name['wpt_size']._options = None
  _REPORTWPT.fields_by_name['wpt_size']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _REPORTWPT.fields_by_name['data']._options = None
  _REPORTWPT.fields_by_name['data']._serialized_options = b'\242?\026 \002)\000\000\000\000\000\210\303\3001\000\000\000\000\000\210\303@P\036'
  _REPORTWPT._options = None
  _REPORTWPT._serialized_options = b'\242?\006\010!\020 (\004'
  _ROSLAUNCH.fields_by_name['time']._options = None
  _ROSLAUNCH.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _ROSLAUNCH.fields_by_name['local_id']._options = None
  _ROSLAUNCH.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _ROSLAUNCH.fields_by_name['remote_id']._options = None
  _ROSLAUNCH.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _ROSLAUNCH.fields_by_name['index']._options = None
  _ROSLAUNCH.fields_by_name['index']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _ROSLAUNCH.fields_by_name['state']._options = None
  _ROSLAUNCH.fields_by_name['state']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000\000@'
  _ROSLAUNCH._options = None
  _ROSLAUNCH._serialized_options = b'\242?\006\010(\020 (\004'
  _REPORTROSLAUNCH.fields_by_name['time']._options = None
  _REPORTROSLAUNCH.fields_by_name['time']._serialized_options = b'\242?\024 \003)\000\000\000\000e\315\315A1\000\000\000\000e\315\335A'
  _REPORTROSLAUNCH.fields_by_name['local_id']._options = None
  _REPORTROSLAUNCH.fields_by_name['local_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _REPORTROSLAUNCH.fields_by_name['remote_id']._options = None
  _REPORTROSLAUNCH.fields_by_name['remote_id']._serialized_options = b'\242?\024 \000)\000\000\000\000\000\000\000\0001\000\000\000\000\000\000Y@'
  _REPORTROSLAUNCH.fields_by_name['state']._options = None
  _REPORTROSLAUNCH.fields_by_name['state']._serialized_options = b'\242?\026 \000)\000\000\000\000\000\000\360\2771\000\000\000\000\000\000\360?P\n'
  _REPORTROSLAUNCH._options = None
  _REPORTROSLAUNCH._serialized_options = b'\242?\006\010)\020 (\004'
  _JOY._serialized_start=53
  _JOY._serialized_end=280
  _PWM._serialized_start=283
  _PWM._serialized_end=504
  _ODOMETRY._serialized_start=507
  _ODOMETRY._serialized_end=883
  _GEOPOSE._serialized_start=886
  _GEOPOSE._serialized_end=1145
  _SETPOWERPORT._serialized_start=1148
  _SETPOWERPORT._serialized_end=1379
  _REPORTPOWERPORT._serialized_start=1382
  _REPORTPOWERPORT._serialized_end=1578
  _SETCONTROLLER._serialized_start=1581
  _SETCONTROLLER._serialized_end=1774
  _REPORTCONTROLLER._serialized_start=1777
  _REPORTCONTROLLER._serialized_end=1973
  _SETHELM._serialized_start=1976
  _SETHELM._serialized_end=2144
  _REPORTHELM._serialized_start=2147
  _REPORTHELM._serialized_end=2350
  _SETWPT._serialized_start=2353
  _SETWPT._serialized_end=2582
  _REPORTWPT._serialized_start=2585
  _REPORTWPT._serialized_end=2817
  _ROSLAUNCH._serialized_start=2820
  _ROSLAUNCH._serialized_end=3048
  _REPORTROSLAUNCH._serialized_start=3051
  _REPORTROSLAUNCH._serialized_end=3247
# @@protoc_insertion_point(module_scope)
