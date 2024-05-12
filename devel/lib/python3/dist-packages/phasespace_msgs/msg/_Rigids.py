# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from phasespace_msgs/Rigids.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import phasespace_msgs.msg
import std_msgs.msg

class Rigids(genpy.Message):
  _md5sum = "1db17ace582e60bc2646dffe29b31add"
  _type = "phasespace_msgs/Rigids"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """########################################
# Messages
########################################
std_msgs/Header header
Rigid[] rigids
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: phasespace_msgs/Rigid
########################################
# Messages
########################################
uint32 id
uint32 flags
uint64 time
float32 x
float32 y
float32 z
float32 qw
float32 qx
float32 qy
float32 qz
float32 cond"""
  __slots__ = ['header','rigids']
  _slot_types = ['std_msgs/Header','phasespace_msgs/Rigid[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,rigids

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Rigids, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.rigids is None:
        self.rigids = []
    else:
      self.header = std_msgs.msg.Header()
      self.rigids = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.rigids)
      buff.write(_struct_I.pack(length))
      for val1 in self.rigids:
        _x = val1
        buff.write(_get_struct_2IQ8f().pack(_x.id, _x.flags, _x.time, _x.x, _x.y, _x.z, _x.qw, _x.qx, _x.qy, _x.qz, _x.cond))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.rigids is None:
        self.rigids = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.rigids = []
      for i in range(0, length):
        val1 = phasespace_msgs.msg.Rigid()
        _x = val1
        start = end
        end += 48
        (_x.id, _x.flags, _x.time, _x.x, _x.y, _x.z, _x.qw, _x.qx, _x.qy, _x.qz, _x.cond,) = _get_struct_2IQ8f().unpack(str[start:end])
        self.rigids.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.rigids)
      buff.write(_struct_I.pack(length))
      for val1 in self.rigids:
        _x = val1
        buff.write(_get_struct_2IQ8f().pack(_x.id, _x.flags, _x.time, _x.x, _x.y, _x.z, _x.qw, _x.qx, _x.qy, _x.qz, _x.cond))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.rigids is None:
        self.rigids = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.rigids = []
      for i in range(0, length):
        val1 = phasespace_msgs.msg.Rigid()
        _x = val1
        start = end
        end += 48
        (_x.id, _x.flags, _x.time, _x.x, _x.y, _x.z, _x.qw, _x.qx, _x.qy, _x.qz, _x.cond,) = _get_struct_2IQ8f().unpack(str[start:end])
        self.rigids.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2IQ8f = None
def _get_struct_2IQ8f():
    global _struct_2IQ8f
    if _struct_2IQ8f is None:
        _struct_2IQ8f = struct.Struct("<2IQ8f")
    return _struct_2IQ8f
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
