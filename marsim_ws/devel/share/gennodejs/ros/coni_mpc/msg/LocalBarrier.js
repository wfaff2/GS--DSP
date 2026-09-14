// Auto-generated. Do not edit!

// (in-package coni_mpc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class LocalBarrier {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.query_center_N = null;
      this.A_row_major = null;
      this.b = null;
      this.c = null;
      this.rmse = null;
      this.max_abs_error = null;
      this.condition_number = null;
      this.rank = null;
      this.min_singular_value = null;
      this.max_singular_value = null;
      this.query_count = null;
      this.valid = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('query_center_N')) {
        this.query_center_N = initObj.query_center_N
      }
      else {
        this.query_center_N = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('A_row_major')) {
        this.A_row_major = initObj.A_row_major
      }
      else {
        this.A_row_major = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('b')) {
        this.b = initObj.b
      }
      else {
        this.b = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('c')) {
        this.c = initObj.c
      }
      else {
        this.c = 0.0;
      }
      if (initObj.hasOwnProperty('rmse')) {
        this.rmse = initObj.rmse
      }
      else {
        this.rmse = 0.0;
      }
      if (initObj.hasOwnProperty('max_abs_error')) {
        this.max_abs_error = initObj.max_abs_error
      }
      else {
        this.max_abs_error = 0.0;
      }
      if (initObj.hasOwnProperty('condition_number')) {
        this.condition_number = initObj.condition_number
      }
      else {
        this.condition_number = 0.0;
      }
      if (initObj.hasOwnProperty('rank')) {
        this.rank = initObj.rank
      }
      else {
        this.rank = 0;
      }
      if (initObj.hasOwnProperty('min_singular_value')) {
        this.min_singular_value = initObj.min_singular_value
      }
      else {
        this.min_singular_value = 0.0;
      }
      if (initObj.hasOwnProperty('max_singular_value')) {
        this.max_singular_value = initObj.max_singular_value
      }
      else {
        this.max_singular_value = 0.0;
      }
      if (initObj.hasOwnProperty('query_count')) {
        this.query_count = initObj.query_count
      }
      else {
        this.query_count = 0;
      }
      if (initObj.hasOwnProperty('valid')) {
        this.valid = initObj.valid
      }
      else {
        this.valid = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LocalBarrier
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [query_center_N]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.query_center_N, buffer, bufferOffset);
    // Check that the constant length array field [A_row_major] has the right length
    if (obj.A_row_major.length !== 9) {
      throw new Error('Unable to serialize array field A_row_major - length must be 9')
    }
    // Serialize message field [A_row_major]
    bufferOffset = _arraySerializer.float64(obj.A_row_major, buffer, bufferOffset, 9);
    // Serialize message field [b]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.b, buffer, bufferOffset);
    // Serialize message field [c]
    bufferOffset = _serializer.float64(obj.c, buffer, bufferOffset);
    // Serialize message field [rmse]
    bufferOffset = _serializer.float64(obj.rmse, buffer, bufferOffset);
    // Serialize message field [max_abs_error]
    bufferOffset = _serializer.float64(obj.max_abs_error, buffer, bufferOffset);
    // Serialize message field [condition_number]
    bufferOffset = _serializer.float64(obj.condition_number, buffer, bufferOffset);
    // Serialize message field [rank]
    bufferOffset = _serializer.int32(obj.rank, buffer, bufferOffset);
    // Serialize message field [min_singular_value]
    bufferOffset = _serializer.float64(obj.min_singular_value, buffer, bufferOffset);
    // Serialize message field [max_singular_value]
    bufferOffset = _serializer.float64(obj.max_singular_value, buffer, bufferOffset);
    // Serialize message field [query_count]
    bufferOffset = _serializer.uint32(obj.query_count, buffer, bufferOffset);
    // Serialize message field [valid]
    bufferOffset = _serializer.bool(obj.valid, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LocalBarrier
    let len;
    let data = new LocalBarrier(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [query_center_N]
    data.query_center_N = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [A_row_major]
    data.A_row_major = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [b]
    data.b = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [c]
    data.c = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rmse]
    data.rmse = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_abs_error]
    data.max_abs_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [condition_number]
    data.condition_number = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rank]
    data.rank = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [min_singular_value]
    data.min_singular_value = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_singular_value]
    data.max_singular_value = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [query_count]
    data.query_count = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [valid]
    data.valid = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 177;
  }

  static datatype() {
    // Returns string type for a message object
    return 'coni_mpc/LocalBarrier';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '582aa024af4e145e9928e13dd367c1d7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    geometry_msgs/Point query_center_N
    float64[9] A_row_major
    geometry_msgs/Vector3 b
    float64 c
    float64 rmse
    float64 max_abs_error
    float64 condition_number
    int32 rank
    float64 min_singular_value
    float64 max_singular_value
    uint32 query_count
    bool valid
    
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
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
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
    const resolved = new LocalBarrier(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.query_center_N !== undefined) {
      resolved.query_center_N = geometry_msgs.msg.Point.Resolve(msg.query_center_N)
    }
    else {
      resolved.query_center_N = new geometry_msgs.msg.Point()
    }

    if (msg.A_row_major !== undefined) {
      resolved.A_row_major = msg.A_row_major;
    }
    else {
      resolved.A_row_major = new Array(9).fill(0)
    }

    if (msg.b !== undefined) {
      resolved.b = geometry_msgs.msg.Vector3.Resolve(msg.b)
    }
    else {
      resolved.b = new geometry_msgs.msg.Vector3()
    }

    if (msg.c !== undefined) {
      resolved.c = msg.c;
    }
    else {
      resolved.c = 0.0
    }

    if (msg.rmse !== undefined) {
      resolved.rmse = msg.rmse;
    }
    else {
      resolved.rmse = 0.0
    }

    if (msg.max_abs_error !== undefined) {
      resolved.max_abs_error = msg.max_abs_error;
    }
    else {
      resolved.max_abs_error = 0.0
    }

    if (msg.condition_number !== undefined) {
      resolved.condition_number = msg.condition_number;
    }
    else {
      resolved.condition_number = 0.0
    }

    if (msg.rank !== undefined) {
      resolved.rank = msg.rank;
    }
    else {
      resolved.rank = 0
    }

    if (msg.min_singular_value !== undefined) {
      resolved.min_singular_value = msg.min_singular_value;
    }
    else {
      resolved.min_singular_value = 0.0
    }

    if (msg.max_singular_value !== undefined) {
      resolved.max_singular_value = msg.max_singular_value;
    }
    else {
      resolved.max_singular_value = 0.0
    }

    if (msg.query_count !== undefined) {
      resolved.query_count = msg.query_count;
    }
    else {
      resolved.query_count = 0
    }

    if (msg.valid !== undefined) {
      resolved.valid = msg.valid;
    }
    else {
      resolved.valid = false
    }

    return resolved;
    }
};

module.exports = LocalBarrier;
