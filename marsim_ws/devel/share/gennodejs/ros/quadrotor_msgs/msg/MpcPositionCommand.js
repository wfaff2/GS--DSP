// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PositionCommand = require('./PositionCommand.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MpcPositionCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.cmds = null;
      this.mpc_horizon = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('cmds')) {
        this.cmds = initObj.cmds
      }
      else {
        this.cmds = [];
      }
      if (initObj.hasOwnProperty('mpc_horizon')) {
        this.mpc_horizon = initObj.mpc_horizon
      }
      else {
        this.mpc_horizon = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MpcPositionCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [cmds]
    // Serialize the length for message field [cmds]
    bufferOffset = _serializer.uint32(obj.cmds.length, buffer, bufferOffset);
    obj.cmds.forEach((val) => {
      bufferOffset = PositionCommand.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [mpc_horizon]
    bufferOffset = _serializer.uint32(obj.mpc_horizon, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MpcPositionCommand
    let len;
    let data = new MpcPositionCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [cmds]
    // Deserialize array length for message field [cmds]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cmds = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cmds[i] = PositionCommand.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [mpc_horizon]
    data.mpc_horizon = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.cmds.forEach((val) => {
      length += PositionCommand.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/MpcPositionCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a72f7a7a678d7a5d422b50d1b2b4f694';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    quadrotor_msgs/PositionCommand[] cmds
    uint32 mpc_horizon
    
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
    MSG: quadrotor_msgs/PositionCommand
    Header header
    geometry_msgs/Point position
    geometry_msgs/Vector3 velocity
    geometry_msgs/Vector3 acceleration
    geometry_msgs/Vector3 jerk
    geometry_msgs/Vector3 angular_velocity
    geometry_msgs/Vector3 attitude
    geometry_msgs/Vector3 thrust
    float64 yaw
    float64 yaw_dot
    float64 vel_norm
    float64 acc_norm
    
    
    float64[3] kx
    float64[3] kv
    uint32 trajectory_id
    uint8 TRAJECTORY_STATUS_EMPTY = 0
    uint8 TRAJECTORY_STATUS_EMER = 2
    uint8 TRAJECTORY_STATUS_READY = 1
    uint8 TRAJECTORY_STATUS_COMPLETED = 3
    uint8 TRAJECTROY_STATUS_ABORT = 4
    uint8 TRAJECTORY_STATUS_ILLEGAL_START = 5
    uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6
    uint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7
    uint32 ACTION_STOP                 =   8
    # Its ID number will start from 1, allowing you comparing it with 0.
    uint8 trajectory_flag
    
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
    const resolved = new MpcPositionCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.cmds !== undefined) {
      resolved.cmds = new Array(msg.cmds.length);
      for (let i = 0; i < resolved.cmds.length; ++i) {
        resolved.cmds[i] = PositionCommand.Resolve(msg.cmds[i]);
      }
    }
    else {
      resolved.cmds = []
    }

    if (msg.mpc_horizon !== undefined) {
      resolved.mpc_horizon = msg.mpc_horizon;
    }
    else {
      resolved.mpc_horizon = 0
    }

    return resolved;
    }
};

module.exports = MpcPositionCommand;
