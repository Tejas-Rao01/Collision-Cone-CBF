// Auto-generated. Do not edit!

// (in-package phasespace_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Camera = require('./Camera.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Cameras {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.cameras = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('cameras')) {
        this.cameras = initObj.cameras
      }
      else {
        this.cameras = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Cameras
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [cameras]
    // Serialize the length for message field [cameras]
    bufferOffset = _serializer.uint32(obj.cameras.length, buffer, bufferOffset);
    obj.cameras.forEach((val) => {
      bufferOffset = Camera.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Cameras
    let len;
    let data = new Cameras(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [cameras]
    // Deserialize array length for message field [cameras]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cameras = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cameras[i] = Camera.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 40 * object.cameras.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'phasespace_msgs/Cameras';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f4b3dddb1243eb3e913cbf3e1940fe8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ########################################
    # Messages
    ########################################
    std_msgs/Header header
    Camera[] cameras
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
    MSG: phasespace_msgs/Camera
    ########################################
    # Messages
    ########################################
    uint32 id
    uint32 flags
    float32 x
    float32 y
    float32 z
    float32 qw
    float32 qx
    float32 qy
    float32 qz
    float32 cond
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Cameras(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.cameras !== undefined) {
      resolved.cameras = new Array(msg.cameras.length);
      for (let i = 0; i < resolved.cameras.length; ++i) {
        resolved.cameras[i] = Camera.Resolve(msg.cameras[i]);
      }
    }
    else {
      resolved.cameras = []
    }

    return resolved;
    }
};

module.exports = Cameras;
