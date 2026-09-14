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

class aec {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.sensor_pos = null;
      this.shutter_val = null;
      this.dgain_val = null;
      this.dgain_a = null;
      this.again_val = null;
      this.again_a = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('sensor_pos')) {
        this.sensor_pos = initObj.sensor_pos
      }
      else {
        this.sensor_pos = 0;
      }
      if (initObj.hasOwnProperty('shutter_val')) {
        this.shutter_val = initObj.shutter_val
      }
      else {
        this.shutter_val = 0;
      }
      if (initObj.hasOwnProperty('dgain_val')) {
        this.dgain_val = initObj.dgain_val
      }
      else {
        this.dgain_val = 0;
      }
      if (initObj.hasOwnProperty('dgain_a')) {
        this.dgain_a = initObj.dgain_a
      }
      else {
        this.dgain_a = 0;
      }
      if (initObj.hasOwnProperty('again_val')) {
        this.again_val = initObj.again_val
      }
      else {
        this.again_val = 0;
      }
      if (initObj.hasOwnProperty('again_a')) {
        this.again_a = initObj.again_a
      }
      else {
        this.again_a = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type aec
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [sensor_pos]
    bufferOffset = _serializer.int16(obj.sensor_pos, buffer, bufferOffset);
    // Serialize message field [shutter_val]
    bufferOffset = _serializer.int16(obj.shutter_val, buffer, bufferOffset);
    // Serialize message field [dgain_val]
    bufferOffset = _serializer.int16(obj.dgain_val, buffer, bufferOffset);
    // Serialize message field [dgain_a]
    bufferOffset = _serializer.int16(obj.dgain_a, buffer, bufferOffset);
    // Serialize message field [again_val]
    bufferOffset = _serializer.int16(obj.again_val, buffer, bufferOffset);
    // Serialize message field [again_a]
    bufferOffset = _serializer.int16(obj.again_a, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type aec
    let len;
    let data = new aec(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [sensor_pos]
    data.sensor_pos = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [shutter_val]
    data.shutter_val = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [dgain_val]
    data.dgain_val = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [dgain_a]
    data.dgain_a = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [again_val]
    data.again_val = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [again_a]
    data.again_a = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/aec';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bb5f8b724b61c9413d80e9b092077813';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    int16 sensor_pos
    int16 shutter_val
    int16 dgain_val
    int16 dgain_a
    int16 again_val
    int16 again_a
    
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
    const resolved = new aec(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.sensor_pos !== undefined) {
      resolved.sensor_pos = msg.sensor_pos;
    }
    else {
      resolved.sensor_pos = 0
    }

    if (msg.shutter_val !== undefined) {
      resolved.shutter_val = msg.shutter_val;
    }
    else {
      resolved.shutter_val = 0
    }

    if (msg.dgain_val !== undefined) {
      resolved.dgain_val = msg.dgain_val;
    }
    else {
      resolved.dgain_val = 0
    }

    if (msg.dgain_a !== undefined) {
      resolved.dgain_a = msg.dgain_a;
    }
    else {
      resolved.dgain_a = 0
    }

    if (msg.again_val !== undefined) {
      resolved.again_val = msg.again_val;
    }
    else {
      resolved.again_val = 0
    }

    if (msg.again_a !== undefined) {
      resolved.again_a = msg.again_a;
    }
    else {
      resolved.again_a = 0
    }

    return resolved;
    }
};

module.exports = aec;
