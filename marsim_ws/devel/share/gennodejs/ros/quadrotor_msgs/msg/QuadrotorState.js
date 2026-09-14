// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class QuadrotorState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.thrust = null;
      this.velocity_norm = null;
      this.acceleration_norm = null;
      this.jerk_norm = null;
      this.position = null;
      this.velocity = null;
      this.acceleration = null;
      this.jerk = null;
      this.snap = null;
      this.attitude = null;
      this.angular_velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('thrust')) {
        this.thrust = initObj.thrust
      }
      else {
        this.thrust = 0.0;
      }
      if (initObj.hasOwnProperty('velocity_norm')) {
        this.velocity_norm = initObj.velocity_norm
      }
      else {
        this.velocity_norm = 0.0;
      }
      if (initObj.hasOwnProperty('acceleration_norm')) {
        this.acceleration_norm = initObj.acceleration_norm
      }
      else {
        this.acceleration_norm = 0.0;
      }
      if (initObj.hasOwnProperty('jerk_norm')) {
        this.jerk_norm = initObj.jerk_norm
      }
      else {
        this.jerk_norm = 0.0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('jerk')) {
        this.jerk = initObj.jerk
      }
      else {
        this.jerk = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('snap')) {
        this.snap = initObj.snap
      }
      else {
        this.snap = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('attitude')) {
        this.attitude = initObj.attitude
      }
      else {
        this.attitude = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('angular_velocity')) {
        this.angular_velocity = initObj.angular_velocity
      }
      else {
        this.angular_velocity = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QuadrotorState
    // Serialize message field [thrust]
    bufferOffset = _serializer.float64(obj.thrust, buffer, bufferOffset);
    // Serialize message field [velocity_norm]
    bufferOffset = _serializer.float64(obj.velocity_norm, buffer, bufferOffset);
    // Serialize message field [acceleration_norm]
    bufferOffset = _serializer.float64(obj.acceleration_norm, buffer, bufferOffset);
    // Serialize message field [jerk_norm]
    bufferOffset = _serializer.float64(obj.jerk_norm, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity, buffer, bufferOffset);
    // Serialize message field [acceleration]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.acceleration, buffer, bufferOffset);
    // Serialize message field [jerk]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.jerk, buffer, bufferOffset);
    // Serialize message field [snap]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.snap, buffer, bufferOffset);
    // Serialize message field [attitude]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.attitude, buffer, bufferOffset);
    // Serialize message field [angular_velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.angular_velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QuadrotorState
    let len;
    let data = new QuadrotorState(null);
    // Deserialize message field [thrust]
    data.thrust = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [velocity_norm]
    data.velocity_norm = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [acceleration_norm]
    data.acceleration_norm = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [jerk_norm]
    data.jerk_norm = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [acceleration]
    data.acceleration = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [jerk]
    data.jerk = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [snap]
    data.snap = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [attitude]
    data.attitude = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [angular_velocity]
    data.angular_velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 200;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/QuadrotorState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9f6239376b083d84c39413d50a0b4367';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new QuadrotorState(null);
    if (msg.thrust !== undefined) {
      resolved.thrust = msg.thrust;
    }
    else {
      resolved.thrust = 0.0
    }

    if (msg.velocity_norm !== undefined) {
      resolved.velocity_norm = msg.velocity_norm;
    }
    else {
      resolved.velocity_norm = 0.0
    }

    if (msg.acceleration_norm !== undefined) {
      resolved.acceleration_norm = msg.acceleration_norm;
    }
    else {
      resolved.acceleration_norm = 0.0
    }

    if (msg.jerk_norm !== undefined) {
      resolved.jerk_norm = msg.jerk_norm;
    }
    else {
      resolved.jerk_norm = 0.0
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = geometry_msgs.msg.Vector3.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.acceleration !== undefined) {
      resolved.acceleration = geometry_msgs.msg.Vector3.Resolve(msg.acceleration)
    }
    else {
      resolved.acceleration = new geometry_msgs.msg.Vector3()
    }

    if (msg.jerk !== undefined) {
      resolved.jerk = geometry_msgs.msg.Vector3.Resolve(msg.jerk)
    }
    else {
      resolved.jerk = new geometry_msgs.msg.Vector3()
    }

    if (msg.snap !== undefined) {
      resolved.snap = geometry_msgs.msg.Vector3.Resolve(msg.snap)
    }
    else {
      resolved.snap = new geometry_msgs.msg.Vector3()
    }

    if (msg.attitude !== undefined) {
      resolved.attitude = geometry_msgs.msg.Vector3.Resolve(msg.attitude)
    }
    else {
      resolved.attitude = new geometry_msgs.msg.Vector3()
    }

    if (msg.angular_velocity !== undefined) {
      resolved.angular_velocity = geometry_msgs.msg.Vector3.Resolve(msg.angular_velocity)
    }
    else {
      resolved.angular_velocity = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = QuadrotorState;
