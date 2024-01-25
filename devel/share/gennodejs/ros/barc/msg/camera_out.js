// Auto-generated. Do not edit!

// (in-package barc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class camera_out {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.s = null;
      this.curv_s = null;
      this.ey = null;
    }
    else {
      if (initObj.hasOwnProperty('s')) {
        this.s = initObj.s
      }
      else {
        this.s = [];
      }
      if (initObj.hasOwnProperty('curv_s')) {
        this.curv_s = initObj.curv_s
      }
      else {
        this.curv_s = [];
      }
      if (initObj.hasOwnProperty('ey')) {
        this.ey = initObj.ey
      }
      else {
        this.ey = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type camera_out
    // Serialize message field [s]
    // Serialize the length for message field [s]
    bufferOffset = _serializer.uint32(obj.s.length, buffer, bufferOffset);
    obj.s.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [curv_s]
    bufferOffset = _arraySerializer.float64(obj.curv_s, buffer, bufferOffset, null);
    // Serialize message field [ey]
    bufferOffset = _serializer.float64(obj.ey, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type camera_out
    let len;
    let data = new camera_out(null);
    // Deserialize message field [s]
    // Deserialize array length for message field [s]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.s = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.s[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [curv_s]
    data.curv_s = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [ey]
    data.ey = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.s.length;
    length += 8 * object.curv_s.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'barc/camera_out';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '20a2606aec8bfaba63eeafd610f117a5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point[] s
    float64[] curv_s
    float64 ey
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new camera_out(null);
    if (msg.s !== undefined) {
      resolved.s = new Array(msg.s.length);
      for (let i = 0; i < resolved.s.length; ++i) {
        resolved.s[i] = geometry_msgs.msg.Point.Resolve(msg.s[i]);
      }
    }
    else {
      resolved.s = []
    }

    if (msg.curv_s !== undefined) {
      resolved.curv_s = msg.curv_s;
    }
    else {
      resolved.curv_s = []
    }

    if (msg.ey !== undefined) {
      resolved.ey = msg.ey;
    }
    else {
      resolved.ey = 0.0
    }

    return resolved;
    }
};

module.exports = camera_out;
