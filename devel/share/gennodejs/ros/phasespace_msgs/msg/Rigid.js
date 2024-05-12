// Auto-generated. Do not edit!

// (in-package phasespace_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Rigid {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.flags = null;
      this.time = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.qw = null;
      this.qx = null;
      this.qy = null;
      this.qz = null;
      this.cond = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('flags')) {
        this.flags = initObj.flags
      }
      else {
        this.flags = 0;
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('qw')) {
        this.qw = initObj.qw
      }
      else {
        this.qw = 0.0;
      }
      if (initObj.hasOwnProperty('qx')) {
        this.qx = initObj.qx
      }
      else {
        this.qx = 0.0;
      }
      if (initObj.hasOwnProperty('qy')) {
        this.qy = initObj.qy
      }
      else {
        this.qy = 0.0;
      }
      if (initObj.hasOwnProperty('qz')) {
        this.qz = initObj.qz
      }
      else {
        this.qz = 0.0;
      }
      if (initObj.hasOwnProperty('cond')) {
        this.cond = initObj.cond
      }
      else {
        this.cond = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Rigid
    // Serialize message field [id]
    bufferOffset = _serializer.uint32(obj.id, buffer, bufferOffset);
    // Serialize message field [flags]
    bufferOffset = _serializer.uint32(obj.flags, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.uint64(obj.time, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float32(obj.z, buffer, bufferOffset);
    // Serialize message field [qw]
    bufferOffset = _serializer.float32(obj.qw, buffer, bufferOffset);
    // Serialize message field [qx]
    bufferOffset = _serializer.float32(obj.qx, buffer, bufferOffset);
    // Serialize message field [qy]
    bufferOffset = _serializer.float32(obj.qy, buffer, bufferOffset);
    // Serialize message field [qz]
    bufferOffset = _serializer.float32(obj.qz, buffer, bufferOffset);
    // Serialize message field [cond]
    bufferOffset = _serializer.float32(obj.cond, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Rigid
    let len;
    let data = new Rigid(null);
    // Deserialize message field [id]
    data.id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [flags]
    data.flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [qw]
    data.qw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [qx]
    data.qx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [qy]
    data.qy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [qz]
    data.qz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cond]
    data.cond = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'phasespace_msgs/Rigid';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '38b7ebb6b44ca41939e6c9ef2ebb6326';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    float32 cond
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Rigid(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.flags !== undefined) {
      resolved.flags = msg.flags;
    }
    else {
      resolved.flags = 0
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.qw !== undefined) {
      resolved.qw = msg.qw;
    }
    else {
      resolved.qw = 0.0
    }

    if (msg.qx !== undefined) {
      resolved.qx = msg.qx;
    }
    else {
      resolved.qx = 0.0
    }

    if (msg.qy !== undefined) {
      resolved.qy = msg.qy;
    }
    else {
      resolved.qy = 0.0
    }

    if (msg.qz !== undefined) {
      resolved.qz = msg.qz;
    }
    else {
      resolved.qz = 0.0
    }

    if (msg.cond !== undefined) {
      resolved.cond = msg.cond;
    }
    else {
      resolved.cond = 0.0
    }

    return resolved;
    }
};

module.exports = Rigid;
