// Auto-generated. Do not edit!

// (in-package local_sensing_node.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class DynamicObstacleState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.obstacle_id = null;
      this.is_dynamic = null;
      this.geometry_type = null;
      this.position = null;
      this.velocity = null;
      this.size = null;
    }
    else {
      if (initObj.hasOwnProperty('obstacle_id')) {
        this.obstacle_id = initObj.obstacle_id
      }
      else {
        this.obstacle_id = 0;
      }
      if (initObj.hasOwnProperty('is_dynamic')) {
        this.is_dynamic = initObj.is_dynamic
      }
      else {
        this.is_dynamic = false;
      }
      if (initObj.hasOwnProperty('geometry_type')) {
        this.geometry_type = initObj.geometry_type
      }
      else {
        this.geometry_type = '';
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
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DynamicObstacleState
    // Serialize message field [obstacle_id]
    bufferOffset = _serializer.int32(obj.obstacle_id, buffer, bufferOffset);
    // Serialize message field [is_dynamic]
    bufferOffset = _serializer.bool(obj.is_dynamic, buffer, bufferOffset);
    // Serialize message field [geometry_type]
    bufferOffset = _serializer.string(obj.geometry_type, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.size, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DynamicObstacleState
    let len;
    let data = new DynamicObstacleState(null);
    // Deserialize message field [obstacle_id]
    data.obstacle_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [is_dynamic]
    data.is_dynamic = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [geometry_type]
    data.geometry_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.geometry_type);
    return length + 81;
  }

  static datatype() {
    // Returns string type for a message object
    return 'local_sensing_node/DynamicObstacleState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd1cd6ca51f05588daa1bdc717cdb87ba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 obstacle_id
    bool is_dynamic
    string geometry_type
    geometry_msgs/Point position
    geometry_msgs/Vector3 velocity
    geometry_msgs/Vector3 size
    
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
    const resolved = new DynamicObstacleState(null);
    if (msg.obstacle_id !== undefined) {
      resolved.obstacle_id = msg.obstacle_id;
    }
    else {
      resolved.obstacle_id = 0
    }

    if (msg.is_dynamic !== undefined) {
      resolved.is_dynamic = msg.is_dynamic;
    }
    else {
      resolved.is_dynamic = false
    }

    if (msg.geometry_type !== undefined) {
      resolved.geometry_type = msg.geometry_type;
    }
    else {
      resolved.geometry_type = ''
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

    if (msg.size !== undefined) {
      resolved.size = geometry_msgs.msg.Vector3.Resolve(msg.size)
    }
    else {
      resolved.size = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = DynamicObstacleState;
