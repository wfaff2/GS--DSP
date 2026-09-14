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

class ctrl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.x_axis_ctrl = null;
      this.y_axis_ctrl = null;
      this.z_axis_ctrl = null;
      this.vertical_ctrl = null;
      this.x_feed_forward = null;
      this.y_feed_forward = null;
      this.flag = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('x_axis_ctrl')) {
        this.x_axis_ctrl = initObj.x_axis_ctrl
      }
      else {
        this.x_axis_ctrl = 0.0;
      }
      if (initObj.hasOwnProperty('y_axis_ctrl')) {
        this.y_axis_ctrl = initObj.y_axis_ctrl
      }
      else {
        this.y_axis_ctrl = 0.0;
      }
      if (initObj.hasOwnProperty('z_axis_ctrl')) {
        this.z_axis_ctrl = initObj.z_axis_ctrl
      }
      else {
        this.z_axis_ctrl = 0.0;
      }
      if (initObj.hasOwnProperty('vertical_ctrl')) {
        this.vertical_ctrl = initObj.vertical_ctrl
      }
      else {
        this.vertical_ctrl = 0.0;
      }
      if (initObj.hasOwnProperty('x_feed_forward')) {
        this.x_feed_forward = initObj.x_feed_forward
      }
      else {
        this.x_feed_forward = 0.0;
      }
      if (initObj.hasOwnProperty('y_feed_forward')) {
        this.y_feed_forward = initObj.y_feed_forward
      }
      else {
        this.y_feed_forward = 0.0;
      }
      if (initObj.hasOwnProperty('flag')) {
        this.flag = initObj.flag
      }
      else {
        this.flag = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ctrl
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [x_axis_ctrl]
    bufferOffset = _serializer.float32(obj.x_axis_ctrl, buffer, bufferOffset);
    // Serialize message field [y_axis_ctrl]
    bufferOffset = _serializer.float32(obj.y_axis_ctrl, buffer, bufferOffset);
    // Serialize message field [z_axis_ctrl]
    bufferOffset = _serializer.float32(obj.z_axis_ctrl, buffer, bufferOffset);
    // Serialize message field [vertical_ctrl]
    bufferOffset = _serializer.float32(obj.vertical_ctrl, buffer, bufferOffset);
    // Serialize message field [x_feed_forward]
    bufferOffset = _serializer.float32(obj.x_feed_forward, buffer, bufferOffset);
    // Serialize message field [y_feed_forward]
    bufferOffset = _serializer.float32(obj.y_feed_forward, buffer, bufferOffset);
    // Serialize message field [flag]
    bufferOffset = _serializer.uint8(obj.flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ctrl
    let len;
    let data = new ctrl(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [x_axis_ctrl]
    data.x_axis_ctrl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_axis_ctrl]
    data.y_axis_ctrl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z_axis_ctrl]
    data.z_axis_ctrl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vertical_ctrl]
    data.vertical_ctrl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x_feed_forward]
    data.x_feed_forward = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_feed_forward]
    data.y_feed_forward = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [flag]
    data.flag = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/ctrl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0f530cfce16387cbbfb41657d4d60cbc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    float32 x_axis_ctrl
    float32 y_axis_ctrl
    float32 z_axis_ctrl
    float32 vertical_ctrl
    float32 x_feed_forward
    float32 y_feed_forward
    uint8   flag
    uint8 FLAG_TYPE_ATTI_VERT_POS = 18   # atti control
    uint8 FLAG_TYPE_VELO_YAW_RATE = 72   # velocity control
    uint8 FLAG_TYPE_HORIZ_ANG_VERT_VEL_YAW_ANG = 1
    uint8 FLAG_VERTICAL_THRUST_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY = 42
    uint8 FLAG_VERTICAL_POSITION_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY = 26
    
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
    const resolved = new ctrl(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.x_axis_ctrl !== undefined) {
      resolved.x_axis_ctrl = msg.x_axis_ctrl;
    }
    else {
      resolved.x_axis_ctrl = 0.0
    }

    if (msg.y_axis_ctrl !== undefined) {
      resolved.y_axis_ctrl = msg.y_axis_ctrl;
    }
    else {
      resolved.y_axis_ctrl = 0.0
    }

    if (msg.z_axis_ctrl !== undefined) {
      resolved.z_axis_ctrl = msg.z_axis_ctrl;
    }
    else {
      resolved.z_axis_ctrl = 0.0
    }

    if (msg.vertical_ctrl !== undefined) {
      resolved.vertical_ctrl = msg.vertical_ctrl;
    }
    else {
      resolved.vertical_ctrl = 0.0
    }

    if (msg.x_feed_forward !== undefined) {
      resolved.x_feed_forward = msg.x_feed_forward;
    }
    else {
      resolved.x_feed_forward = 0.0
    }

    if (msg.y_feed_forward !== undefined) {
      resolved.y_feed_forward = msg.y_feed_forward;
    }
    else {
      resolved.y_feed_forward = 0.0
    }

    if (msg.flag !== undefined) {
      resolved.flag = msg.flag;
    }
    else {
      resolved.flag = 0
    }

    return resolved;
    }
};

// Constants for message
ctrl.Constants = {
  FLAG_TYPE_ATTI_VERT_POS: 18,
  FLAG_TYPE_VELO_YAW_RATE: 72,
  FLAG_TYPE_HORIZ_ANG_VERT_VEL_YAW_ANG: 1,
  FLAG_VERTICAL_THRUST_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY: 42,
  FLAG_VERTICAL_POSITION_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY: 26,
}

module.exports = ctrl;
