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

class MincoTrajectory {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.trajectory_id = null;
      this.action = null;
      this.num_order = null;
      this.num_segment = null;
      this.start_yaw = null;
      this.final_yaw = null;
      this.init_state = null;
      this.final_state = null;
      this.waypoints_x = null;
      this.waypoints_y = null;
      this.waypoints_z = null;
      this.time = null;
      this.mag_coeff = null;
      this.order = null;
      this.start_time = null;
      this.local_start_t = null;
      this.occ_state = null;
      this.debug_info = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('trajectory_id')) {
        this.trajectory_id = initObj.trajectory_id
      }
      else {
        this.trajectory_id = 0;
      }
      if (initObj.hasOwnProperty('action')) {
        this.action = initObj.action
      }
      else {
        this.action = 0;
      }
      if (initObj.hasOwnProperty('num_order')) {
        this.num_order = initObj.num_order
      }
      else {
        this.num_order = 0;
      }
      if (initObj.hasOwnProperty('num_segment')) {
        this.num_segment = initObj.num_segment
      }
      else {
        this.num_segment = 0;
      }
      if (initObj.hasOwnProperty('start_yaw')) {
        this.start_yaw = initObj.start_yaw
      }
      else {
        this.start_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('final_yaw')) {
        this.final_yaw = initObj.final_yaw
      }
      else {
        this.final_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('init_state')) {
        this.init_state = initObj.init_state
      }
      else {
        this.init_state = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('final_state')) {
        this.final_state = initObj.final_state
      }
      else {
        this.final_state = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('waypoints_x')) {
        this.waypoints_x = initObj.waypoints_x
      }
      else {
        this.waypoints_x = [];
      }
      if (initObj.hasOwnProperty('waypoints_y')) {
        this.waypoints_y = initObj.waypoints_y
      }
      else {
        this.waypoints_y = [];
      }
      if (initObj.hasOwnProperty('waypoints_z')) {
        this.waypoints_z = initObj.waypoints_z
      }
      else {
        this.waypoints_z = [];
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = [];
      }
      if (initObj.hasOwnProperty('mag_coeff')) {
        this.mag_coeff = initObj.mag_coeff
      }
      else {
        this.mag_coeff = 0.0;
      }
      if (initObj.hasOwnProperty('order')) {
        this.order = initObj.order
      }
      else {
        this.order = [];
      }
      if (initObj.hasOwnProperty('start_time')) {
        this.start_time = initObj.start_time
      }
      else {
        this.start_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('local_start_t')) {
        this.local_start_t = initObj.local_start_t
      }
      else {
        this.local_start_t = 0.0;
      }
      if (initObj.hasOwnProperty('occ_state')) {
        this.occ_state = initObj.occ_state
      }
      else {
        this.occ_state = 0;
      }
      if (initObj.hasOwnProperty('debug_info')) {
        this.debug_info = initObj.debug_info
      }
      else {
        this.debug_info = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MincoTrajectory
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [trajectory_id]
    bufferOffset = _serializer.uint32(obj.trajectory_id, buffer, bufferOffset);
    // Serialize message field [action]
    bufferOffset = _serializer.uint32(obj.action, buffer, bufferOffset);
    // Serialize message field [num_order]
    bufferOffset = _serializer.uint32(obj.num_order, buffer, bufferOffset);
    // Serialize message field [num_segment]
    bufferOffset = _serializer.uint32(obj.num_segment, buffer, bufferOffset);
    // Serialize message field [start_yaw]
    bufferOffset = _serializer.float64(obj.start_yaw, buffer, bufferOffset);
    // Serialize message field [final_yaw]
    bufferOffset = _serializer.float64(obj.final_yaw, buffer, bufferOffset);
    // Check that the constant length array field [init_state] has the right length
    if (obj.init_state.length !== 12) {
      throw new Error('Unable to serialize array field init_state - length must be 12')
    }
    // Serialize message field [init_state]
    bufferOffset = _arraySerializer.float64(obj.init_state, buffer, bufferOffset, 12);
    // Check that the constant length array field [final_state] has the right length
    if (obj.final_state.length !== 12) {
      throw new Error('Unable to serialize array field final_state - length must be 12')
    }
    // Serialize message field [final_state]
    bufferOffset = _arraySerializer.float64(obj.final_state, buffer, bufferOffset, 12);
    // Serialize message field [waypoints_x]
    bufferOffset = _arraySerializer.float64(obj.waypoints_x, buffer, bufferOffset, null);
    // Serialize message field [waypoints_y]
    bufferOffset = _arraySerializer.float64(obj.waypoints_y, buffer, bufferOffset, null);
    // Serialize message field [waypoints_z]
    bufferOffset = _arraySerializer.float64(obj.waypoints_z, buffer, bufferOffset, null);
    // Serialize message field [time]
    bufferOffset = _arraySerializer.float64(obj.time, buffer, bufferOffset, null);
    // Serialize message field [mag_coeff]
    bufferOffset = _serializer.float64(obj.mag_coeff, buffer, bufferOffset);
    // Serialize message field [order]
    bufferOffset = _arraySerializer.uint32(obj.order, buffer, bufferOffset, null);
    // Serialize message field [start_time]
    bufferOffset = _serializer.time(obj.start_time, buffer, bufferOffset);
    // Serialize message field [local_start_t]
    bufferOffset = _serializer.float64(obj.local_start_t, buffer, bufferOffset);
    // Serialize message field [occ_state]
    bufferOffset = _serializer.uint32(obj.occ_state, buffer, bufferOffset);
    // Serialize message field [debug_info]
    bufferOffset = _serializer.string(obj.debug_info, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MincoTrajectory
    let len;
    let data = new MincoTrajectory(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [trajectory_id]
    data.trajectory_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [action]
    data.action = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [num_order]
    data.num_order = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [num_segment]
    data.num_segment = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [start_yaw]
    data.start_yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [final_yaw]
    data.final_yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [init_state]
    data.init_state = _arrayDeserializer.float64(buffer, bufferOffset, 12)
    // Deserialize message field [final_state]
    data.final_state = _arrayDeserializer.float64(buffer, bufferOffset, 12)
    // Deserialize message field [waypoints_x]
    data.waypoints_x = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [waypoints_y]
    data.waypoints_y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [waypoints_z]
    data.waypoints_z = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [time]
    data.time = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [mag_coeff]
    data.mag_coeff = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [order]
    data.order = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [start_time]
    data.start_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [local_start_t]
    data.local_start_t = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [occ_state]
    data.occ_state = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [debug_info]
    data.debug_info = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.waypoints_x.length;
    length += 8 * object.waypoints_y.length;
    length += 8 * object.waypoints_z.length;
    length += 8 * object.time.length;
    length += 4 * object.order.length;
    length += _getByteLength(object.debug_info);
    return length + 276;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/MincoTrajectory';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '00d62cdfef43fbf8db0e1a3a7f618be6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    # the trajectory id, starts from "1".
    uint32 trajectory_id
    
    # the action command for trajectory server.
    uint32 ACTION_ADD                  =   1
    uint32 ACTION_ABORT                =   2
    uint32 ACTION_WARN_START           =   3
    uint32 ACTION_WARN_FINAL           =   4
    uint32 ACTION_WARN_IMPOSSIBLE      =   5
    uint32 action
    
    # the order of trajectory.
    uint32 num_order
    uint32 num_segment
    
    # the polynomial coecfficients of the trajectory.
    float64 start_yaw
    float64 final_yaw
    float64[12] init_state
    float64[12] final_state
    float64[] waypoints_x
    float64[] waypoints_y
    float64[] waypoints_z
    float64[] time
    float64   mag_coeff
    uint32[]  order
    time start_time
    float64 local_start_t
    #the occ state of trajectory.
    uint32 occ_state
    string debug_info
    
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
    const resolved = new MincoTrajectory(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.trajectory_id !== undefined) {
      resolved.trajectory_id = msg.trajectory_id;
    }
    else {
      resolved.trajectory_id = 0
    }

    if (msg.action !== undefined) {
      resolved.action = msg.action;
    }
    else {
      resolved.action = 0
    }

    if (msg.num_order !== undefined) {
      resolved.num_order = msg.num_order;
    }
    else {
      resolved.num_order = 0
    }

    if (msg.num_segment !== undefined) {
      resolved.num_segment = msg.num_segment;
    }
    else {
      resolved.num_segment = 0
    }

    if (msg.start_yaw !== undefined) {
      resolved.start_yaw = msg.start_yaw;
    }
    else {
      resolved.start_yaw = 0.0
    }

    if (msg.final_yaw !== undefined) {
      resolved.final_yaw = msg.final_yaw;
    }
    else {
      resolved.final_yaw = 0.0
    }

    if (msg.init_state !== undefined) {
      resolved.init_state = msg.init_state;
    }
    else {
      resolved.init_state = new Array(12).fill(0)
    }

    if (msg.final_state !== undefined) {
      resolved.final_state = msg.final_state;
    }
    else {
      resolved.final_state = new Array(12).fill(0)
    }

    if (msg.waypoints_x !== undefined) {
      resolved.waypoints_x = msg.waypoints_x;
    }
    else {
      resolved.waypoints_x = []
    }

    if (msg.waypoints_y !== undefined) {
      resolved.waypoints_y = msg.waypoints_y;
    }
    else {
      resolved.waypoints_y = []
    }

    if (msg.waypoints_z !== undefined) {
      resolved.waypoints_z = msg.waypoints_z;
    }
    else {
      resolved.waypoints_z = []
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = []
    }

    if (msg.mag_coeff !== undefined) {
      resolved.mag_coeff = msg.mag_coeff;
    }
    else {
      resolved.mag_coeff = 0.0
    }

    if (msg.order !== undefined) {
      resolved.order = msg.order;
    }
    else {
      resolved.order = []
    }

    if (msg.start_time !== undefined) {
      resolved.start_time = msg.start_time;
    }
    else {
      resolved.start_time = {secs: 0, nsecs: 0}
    }

    if (msg.local_start_t !== undefined) {
      resolved.local_start_t = msg.local_start_t;
    }
    else {
      resolved.local_start_t = 0.0
    }

    if (msg.occ_state !== undefined) {
      resolved.occ_state = msg.occ_state;
    }
    else {
      resolved.occ_state = 0
    }

    if (msg.debug_info !== undefined) {
      resolved.debug_info = msg.debug_info;
    }
    else {
      resolved.debug_info = ''
    }

    return resolved;
    }
};

// Constants for message
MincoTrajectory.Constants = {
  ACTION_ADD: 1,
  ACTION_ABORT: 2,
  ACTION_WARN_START: 3,
  ACTION_WARN_FINAL: 4,
  ACTION_WARN_IMPOSSIBLE: 5,
}

module.exports = MincoTrajectory;
