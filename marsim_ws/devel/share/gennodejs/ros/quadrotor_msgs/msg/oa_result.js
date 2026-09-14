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

class oa_result {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.direction = null;
      this.reserved = null;
      this.tof_dist = null;
      this.image_index = null;
      this.image_timestamp = null;
      this.object_distance = null;
      this.stop = null;
      this.failed_distance = null;
      this.failed_distance_app = null;
      this.speed_limit = null;
      this.alarm = null;
      this.object_distance_app = null;
      this.high_atti = null;
      this.is_release_brake = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('direction')) {
        this.direction = initObj.direction
      }
      else {
        this.direction = 0;
      }
      if (initObj.hasOwnProperty('reserved')) {
        this.reserved = initObj.reserved
      }
      else {
        this.reserved = 0;
      }
      if (initObj.hasOwnProperty('tof_dist')) {
        this.tof_dist = initObj.tof_dist
      }
      else {
        this.tof_dist = 0.0;
      }
      if (initObj.hasOwnProperty('image_index')) {
        this.image_index = initObj.image_index
      }
      else {
        this.image_index = 0;
      }
      if (initObj.hasOwnProperty('image_timestamp')) {
        this.image_timestamp = initObj.image_timestamp
      }
      else {
        this.image_timestamp = 0;
      }
      if (initObj.hasOwnProperty('object_distance')) {
        this.object_distance = initObj.object_distance
      }
      else {
        this.object_distance = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('stop')) {
        this.stop = initObj.stop
      }
      else {
        this.stop = 0;
      }
      if (initObj.hasOwnProperty('failed_distance')) {
        this.failed_distance = initObj.failed_distance
      }
      else {
        this.failed_distance = 0;
      }
      if (initObj.hasOwnProperty('failed_distance_app')) {
        this.failed_distance_app = initObj.failed_distance_app
      }
      else {
        this.failed_distance_app = 0;
      }
      if (initObj.hasOwnProperty('speed_limit')) {
        this.speed_limit = initObj.speed_limit
      }
      else {
        this.speed_limit = 0;
      }
      if (initObj.hasOwnProperty('alarm')) {
        this.alarm = initObj.alarm
      }
      else {
        this.alarm = 0;
      }
      if (initObj.hasOwnProperty('object_distance_app')) {
        this.object_distance_app = initObj.object_distance_app
      }
      else {
        this.object_distance_app = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('high_atti')) {
        this.high_atti = initObj.high_atti
      }
      else {
        this.high_atti = 0;
      }
      if (initObj.hasOwnProperty('is_release_brake')) {
        this.is_release_brake = initObj.is_release_brake
      }
      else {
        this.is_release_brake = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type oa_result
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [direction]
    bufferOffset = _serializer.uint32(obj.direction, buffer, bufferOffset);
    // Serialize message field [reserved]
    bufferOffset = _serializer.int32(obj.reserved, buffer, bufferOffset);
    // Serialize message field [tof_dist]
    bufferOffset = _serializer.float32(obj.tof_dist, buffer, bufferOffset);
    // Serialize message field [image_index]
    bufferOffset = _serializer.int32(obj.image_index, buffer, bufferOffset);
    // Serialize message field [image_timestamp]
    bufferOffset = _serializer.uint32(obj.image_timestamp, buffer, bufferOffset);
    // Check that the constant length array field [object_distance] has the right length
    if (obj.object_distance.length !== 3) {
      throw new Error('Unable to serialize array field object_distance - length must be 3')
    }
    // Serialize message field [object_distance]
    bufferOffset = _arraySerializer.int16(obj.object_distance, buffer, bufferOffset, 3);
    // Serialize message field [stop]
    bufferOffset = _serializer.uint8(obj.stop, buffer, bufferOffset);
    // Serialize message field [failed_distance]
    bufferOffset = _serializer.uint8(obj.failed_distance, buffer, bufferOffset);
    // Serialize message field [failed_distance_app]
    bufferOffset = _serializer.uint8(obj.failed_distance_app, buffer, bufferOffset);
    // Serialize message field [speed_limit]
    bufferOffset = _serializer.uint8(obj.speed_limit, buffer, bufferOffset);
    // Serialize message field [alarm]
    bufferOffset = _serializer.uint8(obj.alarm, buffer, bufferOffset);
    // Check that the constant length array field [object_distance_app] has the right length
    if (obj.object_distance_app.length !== 4) {
      throw new Error('Unable to serialize array field object_distance_app - length must be 4')
    }
    // Serialize message field [object_distance_app]
    bufferOffset = _arraySerializer.int16(obj.object_distance_app, buffer, bufferOffset, 4);
    // Serialize message field [high_atti]
    bufferOffset = _serializer.int32(obj.high_atti, buffer, bufferOffset);
    // Serialize message field [is_release_brake]
    bufferOffset = _serializer.int32(obj.is_release_brake, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type oa_result
    let len;
    let data = new oa_result(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [direction]
    data.direction = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [reserved]
    data.reserved = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [tof_dist]
    data.tof_dist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [image_index]
    data.image_index = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [image_timestamp]
    data.image_timestamp = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [object_distance]
    data.object_distance = _arrayDeserializer.int16(buffer, bufferOffset, 3)
    // Deserialize message field [stop]
    data.stop = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [failed_distance]
    data.failed_distance = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [failed_distance_app]
    data.failed_distance_app = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [speed_limit]
    data.speed_limit = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [alarm]
    data.alarm = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [object_distance_app]
    data.object_distance_app = _arrayDeserializer.int16(buffer, bufferOffset, 4)
    // Deserialize message field [high_atti]
    data.high_atti = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [is_release_brake]
    data.is_release_brake = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 47;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/oa_result';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1f8843ccbd246ed329e5be927b5cd5cb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    uint32 direction
    int32 reserved
    float32 tof_dist
    int32 image_index
    uint32 image_timestamp
    int16[3] object_distance        # uint8 NUMBER_SUB_DIRECTIONS = 3, for FC
    uint8 stop
    uint8 failed_distance
    uint8 failed_distance_app
    uint8 speed_limit
    uint8 alarm
    int16[4] object_distance_app    # uint8 NUMBER_SUB_DIRECTIONS_FOR_APP = 4, for APP
    int32 high_atti
    int32 is_release_brake
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
    const resolved = new oa_result(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.direction !== undefined) {
      resolved.direction = msg.direction;
    }
    else {
      resolved.direction = 0
    }

    if (msg.reserved !== undefined) {
      resolved.reserved = msg.reserved;
    }
    else {
      resolved.reserved = 0
    }

    if (msg.tof_dist !== undefined) {
      resolved.tof_dist = msg.tof_dist;
    }
    else {
      resolved.tof_dist = 0.0
    }

    if (msg.image_index !== undefined) {
      resolved.image_index = msg.image_index;
    }
    else {
      resolved.image_index = 0
    }

    if (msg.image_timestamp !== undefined) {
      resolved.image_timestamp = msg.image_timestamp;
    }
    else {
      resolved.image_timestamp = 0
    }

    if (msg.object_distance !== undefined) {
      resolved.object_distance = msg.object_distance;
    }
    else {
      resolved.object_distance = new Array(3).fill(0)
    }

    if (msg.stop !== undefined) {
      resolved.stop = msg.stop;
    }
    else {
      resolved.stop = 0
    }

    if (msg.failed_distance !== undefined) {
      resolved.failed_distance = msg.failed_distance;
    }
    else {
      resolved.failed_distance = 0
    }

    if (msg.failed_distance_app !== undefined) {
      resolved.failed_distance_app = msg.failed_distance_app;
    }
    else {
      resolved.failed_distance_app = 0
    }

    if (msg.speed_limit !== undefined) {
      resolved.speed_limit = msg.speed_limit;
    }
    else {
      resolved.speed_limit = 0
    }

    if (msg.alarm !== undefined) {
      resolved.alarm = msg.alarm;
    }
    else {
      resolved.alarm = 0
    }

    if (msg.object_distance_app !== undefined) {
      resolved.object_distance_app = msg.object_distance_app;
    }
    else {
      resolved.object_distance_app = new Array(4).fill(0)
    }

    if (msg.high_atti !== undefined) {
      resolved.high_atti = msg.high_atti;
    }
    else {
      resolved.high_atti = 0
    }

    if (msg.is_release_brake !== undefined) {
      resolved.is_release_brake = msg.is_release_brake;
    }
    else {
      resolved.is_release_brake = 0
    }

    return resolved;
    }
};

module.exports = oa_result;
