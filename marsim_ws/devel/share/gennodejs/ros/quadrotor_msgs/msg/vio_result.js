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
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class vio_result {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.pose_id = null;
      this.time_stamp_ms = null;
      this.flag = null;
      this.p_wi = null;
      this.v_wi = null;
      this.q_wi = null;
      this.q_ic = null;
      this.p_ic = null;
      this.vps_from_vio = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('pose_id')) {
        this.pose_id = initObj.pose_id
      }
      else {
        this.pose_id = 0;
      }
      if (initObj.hasOwnProperty('time_stamp_ms')) {
        this.time_stamp_ms = initObj.time_stamp_ms
      }
      else {
        this.time_stamp_ms = 0;
      }
      if (initObj.hasOwnProperty('flag')) {
        this.flag = initObj.flag
      }
      else {
        this.flag = 0;
      }
      if (initObj.hasOwnProperty('p_wi')) {
        this.p_wi = initObj.p_wi
      }
      else {
        this.p_wi = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('v_wi')) {
        this.v_wi = initObj.v_wi
      }
      else {
        this.v_wi = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('q_wi')) {
        this.q_wi = initObj.q_wi
      }
      else {
        this.q_wi = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('q_ic')) {
        this.q_ic = initObj.q_ic
      }
      else {
        this.q_ic = new Array(6).fill(new geometry_msgs.msg.Quaternion());
      }
      if (initObj.hasOwnProperty('p_ic')) {
        this.p_ic = initObj.p_ic
      }
      else {
        this.p_ic = new Array(6).fill(new geometry_msgs.msg.Vector3());
      }
      if (initObj.hasOwnProperty('vps_from_vio')) {
        this.vps_from_vio = initObj.vps_from_vio
      }
      else {
        this.vps_from_vio = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type vio_result
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [pose_id]
    bufferOffset = _serializer.uint32(obj.pose_id, buffer, bufferOffset);
    // Serialize message field [time_stamp_ms]
    bufferOffset = _serializer.uint32(obj.time_stamp_ms, buffer, bufferOffset);
    // Serialize message field [flag]
    bufferOffset = _serializer.uint8(obj.flag, buffer, bufferOffset);
    // Serialize message field [p_wi]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.p_wi, buffer, bufferOffset);
    // Serialize message field [v_wi]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.v_wi, buffer, bufferOffset);
    // Serialize message field [q_wi]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.q_wi, buffer, bufferOffset);
    // Check that the constant length array field [q_ic] has the right length
    if (obj.q_ic.length !== 6) {
      throw new Error('Unable to serialize array field q_ic - length must be 6')
    }
    // Serialize message field [q_ic]
    obj.q_ic.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Quaternion.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [p_ic] has the right length
    if (obj.p_ic.length !== 6) {
      throw new Error('Unable to serialize array field p_ic - length must be 6')
    }
    // Serialize message field [p_ic]
    obj.p_ic.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [vps_from_vio]
    bufferOffset = _serializer.float32(obj.vps_from_vio, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type vio_result
    let len;
    let data = new vio_result(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose_id]
    data.pose_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [time_stamp_ms]
    data.time_stamp_ms = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [flag]
    data.flag = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [p_wi]
    data.p_wi = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [v_wi]
    data.v_wi = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [q_wi]
    data.q_wi = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [q_ic]
    len = 6;
    data.q_ic = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.q_ic[i] = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [p_ic]
    len = 6;
    data.p_ic = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.p_ic[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [vps_from_vio]
    data.vps_from_vio = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 429;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/vio_result';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '324b3402a89081d3fe2b8181d4ced579';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    uint32 pose_id
    uint32 time_stamp_ms
    uint8 flag
    geometry_msgs/Vector3 p_wi
    geometry_msgs/Vector3 v_wi
    geometry_msgs/Quaternion q_wi
    geometry_msgs/Quaternion[6] q_ic
    geometry_msgs/Vector3[6] p_ic
    float32 vps_from_vio
    
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
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new vio_result(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.pose_id !== undefined) {
      resolved.pose_id = msg.pose_id;
    }
    else {
      resolved.pose_id = 0
    }

    if (msg.time_stamp_ms !== undefined) {
      resolved.time_stamp_ms = msg.time_stamp_ms;
    }
    else {
      resolved.time_stamp_ms = 0
    }

    if (msg.flag !== undefined) {
      resolved.flag = msg.flag;
    }
    else {
      resolved.flag = 0
    }

    if (msg.p_wi !== undefined) {
      resolved.p_wi = geometry_msgs.msg.Vector3.Resolve(msg.p_wi)
    }
    else {
      resolved.p_wi = new geometry_msgs.msg.Vector3()
    }

    if (msg.v_wi !== undefined) {
      resolved.v_wi = geometry_msgs.msg.Vector3.Resolve(msg.v_wi)
    }
    else {
      resolved.v_wi = new geometry_msgs.msg.Vector3()
    }

    if (msg.q_wi !== undefined) {
      resolved.q_wi = geometry_msgs.msg.Quaternion.Resolve(msg.q_wi)
    }
    else {
      resolved.q_wi = new geometry_msgs.msg.Quaternion()
    }

    if (msg.q_ic !== undefined) {
      resolved.q_ic = new Array(6)
      for (let i = 0; i < resolved.q_ic.length; ++i) {
        if (msg.q_ic.length > i) {
          resolved.q_ic[i] = geometry_msgs.msg.Quaternion.Resolve(msg.q_ic[i]);
        }
        else {
          resolved.q_ic[i] = new geometry_msgs.msg.Quaternion();
        }
      }
    }
    else {
      resolved.q_ic = new Array(6).fill(new geometry_msgs.msg.Quaternion())
    }

    if (msg.p_ic !== undefined) {
      resolved.p_ic = new Array(6)
      for (let i = 0; i < resolved.p_ic.length; ++i) {
        if (msg.p_ic.length > i) {
          resolved.p_ic[i] = geometry_msgs.msg.Vector3.Resolve(msg.p_ic[i]);
        }
        else {
          resolved.p_ic[i] = new geometry_msgs.msg.Vector3();
        }
      }
    }
    else {
      resolved.p_ic = new Array(6).fill(new geometry_msgs.msg.Vector3())
    }

    if (msg.vps_from_vio !== undefined) {
      resolved.vps_from_vio = msg.vps_from_vio;
    }
    else {
      resolved.vps_from_vio = 0.0
    }

    return resolved;
    }
};

module.exports = vio_result;
