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

class drone_aec_info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.expo_time_ms = null;
      this.expo_gain = null;
      this.scene_lum_lux = null;
      this.avg_lum = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('expo_time_ms')) {
        this.expo_time_ms = initObj.expo_time_ms
      }
      else {
        this.expo_time_ms = 0.0;
      }
      if (initObj.hasOwnProperty('expo_gain')) {
        this.expo_gain = initObj.expo_gain
      }
      else {
        this.expo_gain = 0.0;
      }
      if (initObj.hasOwnProperty('scene_lum_lux')) {
        this.scene_lum_lux = initObj.scene_lum_lux
      }
      else {
        this.scene_lum_lux = 0.0;
      }
      if (initObj.hasOwnProperty('avg_lum')) {
        this.avg_lum = initObj.avg_lum
      }
      else {
        this.avg_lum = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type drone_aec_info
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [expo_time_ms]
    bufferOffset = _serializer.float32(obj.expo_time_ms, buffer, bufferOffset);
    // Serialize message field [expo_gain]
    bufferOffset = _serializer.float32(obj.expo_gain, buffer, bufferOffset);
    // Serialize message field [scene_lum_lux]
    bufferOffset = _serializer.float32(obj.scene_lum_lux, buffer, bufferOffset);
    // Serialize message field [avg_lum]
    bufferOffset = _serializer.float32(obj.avg_lum, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type drone_aec_info
    let len;
    let data = new drone_aec_info(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [expo_time_ms]
    data.expo_time_ms = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [expo_gain]
    data.expo_gain = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [scene_lum_lux]
    data.scene_lum_lux = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [avg_lum]
    data.avg_lum = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/drone_aec_info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c3d90219ba162118512fb07f93a5de00';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    float32 expo_time_ms
    float32 expo_gain
    float32 scene_lum_lux
    float32 avg_lum
    
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
    const resolved = new drone_aec_info(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.expo_time_ms !== undefined) {
      resolved.expo_time_ms = msg.expo_time_ms;
    }
    else {
      resolved.expo_time_ms = 0.0
    }

    if (msg.expo_gain !== undefined) {
      resolved.expo_gain = msg.expo_gain;
    }
    else {
      resolved.expo_gain = 0.0
    }

    if (msg.scene_lum_lux !== undefined) {
      resolved.scene_lum_lux = msg.scene_lum_lux;
    }
    else {
      resolved.scene_lum_lux = 0.0
    }

    if (msg.avg_lum !== undefined) {
      resolved.avg_lum = msg.avg_lum;
    }
    else {
      resolved.avg_lum = 0.0
    }

    return resolved;
    }
};

module.exports = drone_aec_info;
