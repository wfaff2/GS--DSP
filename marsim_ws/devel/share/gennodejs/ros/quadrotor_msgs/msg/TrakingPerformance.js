// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let QuadrotorState = require('./QuadrotorState.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TrakingPerformance {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.fsm_state = null;
      this.mpc_solve_time = null;
      this.desire = null;
      this.feedback = null;
      this.error = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('fsm_state')) {
        this.fsm_state = initObj.fsm_state
      }
      else {
        this.fsm_state = new std_msgs.msg.Int64();
      }
      if (initObj.hasOwnProperty('mpc_solve_time')) {
        this.mpc_solve_time = initObj.mpc_solve_time
      }
      else {
        this.mpc_solve_time = 0.0;
      }
      if (initObj.hasOwnProperty('desire')) {
        this.desire = initObj.desire
      }
      else {
        this.desire = new QuadrotorState();
      }
      if (initObj.hasOwnProperty('feedback')) {
        this.feedback = initObj.feedback
      }
      else {
        this.feedback = new QuadrotorState();
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = new QuadrotorState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrakingPerformance
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [fsm_state]
    bufferOffset = std_msgs.msg.Int64.serialize(obj.fsm_state, buffer, bufferOffset);
    // Serialize message field [mpc_solve_time]
    bufferOffset = _serializer.float64(obj.mpc_solve_time, buffer, bufferOffset);
    // Serialize message field [desire]
    bufferOffset = QuadrotorState.serialize(obj.desire, buffer, bufferOffset);
    // Serialize message field [feedback]
    bufferOffset = QuadrotorState.serialize(obj.feedback, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = QuadrotorState.serialize(obj.error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrakingPerformance
    let len;
    let data = new TrakingPerformance(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [fsm_state]
    data.fsm_state = std_msgs.msg.Int64.deserialize(buffer, bufferOffset);
    // Deserialize message field [mpc_solve_time]
    data.mpc_solve_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [desire]
    data.desire = QuadrotorState.deserialize(buffer, bufferOffset);
    // Deserialize message field [feedback]
    data.feedback = QuadrotorState.deserialize(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = QuadrotorState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 616;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/TrakingPerformance';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '91224dc465a6f21d706763bf5df5f87e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Header
    std_msgs/Header header
    # MPC finite state machine state
    std_msgs/Int64 fsm_state
    # Solving time consuming
    float64 mpc_solve_time
    
    QuadrotorState desire
    QuadrotorState feedback
    QuadrotorState error
    
    
    
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
    MSG: std_msgs/Int64
    int64 data
    ================================================================================
    MSG: quadrotor_msgs/QuadrotorState
    float64 thrust
    float64 velocity_norm
    float64 acceleration_norm
    float64 jerk_norm
    geometry_msgs/Point position
    geometry_msgs/Vector3 velocity
    geometry_msgs/Vector3 acceleration
    geometry_msgs/Vector3 jerk
    geometry_msgs/Vector3 snap
    geometry_msgs/Vector3 attitude
    geometry_msgs/Vector3 angular_velocity
    
    
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
    const resolved = new TrakingPerformance(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.fsm_state !== undefined) {
      resolved.fsm_state = std_msgs.msg.Int64.Resolve(msg.fsm_state)
    }
    else {
      resolved.fsm_state = new std_msgs.msg.Int64()
    }

    if (msg.mpc_solve_time !== undefined) {
      resolved.mpc_solve_time = msg.mpc_solve_time;
    }
    else {
      resolved.mpc_solve_time = 0.0
    }

    if (msg.desire !== undefined) {
      resolved.desire = QuadrotorState.Resolve(msg.desire)
    }
    else {
      resolved.desire = new QuadrotorState()
    }

    if (msg.feedback !== undefined) {
      resolved.feedback = QuadrotorState.Resolve(msg.feedback)
    }
    else {
      resolved.feedback = new QuadrotorState()
    }

    if (msg.error !== undefined) {
      resolved.error = QuadrotorState.Resolve(msg.error)
    }
    else {
      resolved.error = new QuadrotorState()
    }

    return resolved;
    }
};

module.exports = TrakingPerformance;
