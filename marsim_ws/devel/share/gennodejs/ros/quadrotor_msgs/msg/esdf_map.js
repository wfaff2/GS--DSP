// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class esdf_map {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.map_plantform = null;
      this.map_version = null;
      this.map_log_version = null;
      this.map_log_type = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('map_plantform')) {
        this.map_plantform = initObj.map_plantform
      }
      else {
        this.map_plantform = 0;
      }
      if (initObj.hasOwnProperty('map_version')) {
        this.map_version = initObj.map_version
      }
      else {
        this.map_version = 0;
      }
      if (initObj.hasOwnProperty('map_log_version')) {
        this.map_log_version = initObj.map_log_version
      }
      else {
        this.map_log_version = 0;
      }
      if (initObj.hasOwnProperty('map_log_type')) {
        this.map_log_type = initObj.map_log_type
      }
      else {
        this.map_log_type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type esdf_map
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [map_plantform]
    bufferOffset = _serializer.uint16(obj.map_plantform, buffer, bufferOffset);
    // Serialize message field [map_version]
    bufferOffset = _serializer.uint8(obj.map_version, buffer, bufferOffset);
    // Serialize message field [map_log_version]
    bufferOffset = _serializer.uint8(obj.map_log_version, buffer, bufferOffset);
    // Serialize message field [map_log_type]
    bufferOffset = _serializer.uint8(obj.map_log_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type esdf_map
    let len;
    let data = new esdf_map(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [map_plantform]
    data.map_plantform = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [map_version]
    data.map_version = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [map_log_version]
    data.map_log_version = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [map_log_type]
    data.map_log_type = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/esdf_map';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b7e611753bf33b0e49ba1e83fb044535';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint16 map_plantform
    uint8 map_version
    uint8 map_log_version
    uint8 map_log_type
    
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new esdf_map(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.map_plantform !== undefined) {
      resolved.map_plantform = msg.map_plantform;
    }
    else {
      resolved.map_plantform = 0
    }

    if (msg.map_version !== undefined) {
      resolved.map_version = msg.map_version;
    }
    else {
      resolved.map_version = 0
    }

    if (msg.map_log_version !== undefined) {
      resolved.map_log_version = msg.map_log_version;
    }
    else {
      resolved.map_log_version = 0
    }

    if (msg.map_log_type !== undefined) {
      resolved.map_log_type = msg.map_log_type;
    }
    else {
      resolved.map_log_type = 0
    }

    return resolved;
    }
};

module.exports = esdf_map;
