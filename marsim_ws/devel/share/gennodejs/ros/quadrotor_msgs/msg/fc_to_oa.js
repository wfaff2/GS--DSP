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

class fc_to_oa {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.q0 = null;
      this.q1 = null;
      this.q2 = null;
      this.q3 = null;
      this.gyro_x = null;
      this.gyro_y = null;
      this.gyro_z = null;
      this.acc_x = null;
      this.acc_y = null;
      this.acc_z = null;
      this.down_dist = null;
      this.top_dist = null;
      this.lati = null;
      this.longti = null;
      this.press = null;
      this.vg_x = null;
      this.vg_y = null;
      this.vg_z = null;
      this.aileron = null;
      this.elevator = null;
      this.throttle = null;
      this.rudder = null;
      this.fc_timestamp_ms = null;
      this.vo_flag = null;
      this.roll = null;
      this.pitch = null;
      this.yaw = null;
      this.anti_longti = null;
      this.anti_press = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('q0')) {
        this.q0 = initObj.q0
      }
      else {
        this.q0 = 0.0;
      }
      if (initObj.hasOwnProperty('q1')) {
        this.q1 = initObj.q1
      }
      else {
        this.q1 = 0.0;
      }
      if (initObj.hasOwnProperty('q2')) {
        this.q2 = initObj.q2
      }
      else {
        this.q2 = 0.0;
      }
      if (initObj.hasOwnProperty('q3')) {
        this.q3 = initObj.q3
      }
      else {
        this.q3 = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_x')) {
        this.gyro_x = initObj.gyro_x
      }
      else {
        this.gyro_x = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_y')) {
        this.gyro_y = initObj.gyro_y
      }
      else {
        this.gyro_y = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_z')) {
        this.gyro_z = initObj.gyro_z
      }
      else {
        this.gyro_z = 0.0;
      }
      if (initObj.hasOwnProperty('acc_x')) {
        this.acc_x = initObj.acc_x
      }
      else {
        this.acc_x = 0.0;
      }
      if (initObj.hasOwnProperty('acc_y')) {
        this.acc_y = initObj.acc_y
      }
      else {
        this.acc_y = 0.0;
      }
      if (initObj.hasOwnProperty('acc_z')) {
        this.acc_z = initObj.acc_z
      }
      else {
        this.acc_z = 0.0;
      }
      if (initObj.hasOwnProperty('down_dist')) {
        this.down_dist = initObj.down_dist
      }
      else {
        this.down_dist = 0.0;
      }
      if (initObj.hasOwnProperty('top_dist')) {
        this.top_dist = initObj.top_dist
      }
      else {
        this.top_dist = 0.0;
      }
      if (initObj.hasOwnProperty('lati')) {
        this.lati = initObj.lati
      }
      else {
        this.lati = 0.0;
      }
      if (initObj.hasOwnProperty('longti')) {
        this.longti = initObj.longti
      }
      else {
        this.longti = 0.0;
      }
      if (initObj.hasOwnProperty('press')) {
        this.press = initObj.press
      }
      else {
        this.press = 0.0;
      }
      if (initObj.hasOwnProperty('vg_x')) {
        this.vg_x = initObj.vg_x
      }
      else {
        this.vg_x = 0.0;
      }
      if (initObj.hasOwnProperty('vg_y')) {
        this.vg_y = initObj.vg_y
      }
      else {
        this.vg_y = 0.0;
      }
      if (initObj.hasOwnProperty('vg_z')) {
        this.vg_z = initObj.vg_z
      }
      else {
        this.vg_z = 0.0;
      }
      if (initObj.hasOwnProperty('aileron')) {
        this.aileron = initObj.aileron
      }
      else {
        this.aileron = 0;
      }
      if (initObj.hasOwnProperty('elevator')) {
        this.elevator = initObj.elevator
      }
      else {
        this.elevator = 0;
      }
      if (initObj.hasOwnProperty('throttle')) {
        this.throttle = initObj.throttle
      }
      else {
        this.throttle = 0;
      }
      if (initObj.hasOwnProperty('rudder')) {
        this.rudder = initObj.rudder
      }
      else {
        this.rudder = 0;
      }
      if (initObj.hasOwnProperty('fc_timestamp_ms')) {
        this.fc_timestamp_ms = initObj.fc_timestamp_ms
      }
      else {
        this.fc_timestamp_ms = 0;
      }
      if (initObj.hasOwnProperty('vo_flag')) {
        this.vo_flag = initObj.vo_flag
      }
      else {
        this.vo_flag = 0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('anti_longti')) {
        this.anti_longti = initObj.anti_longti
      }
      else {
        this.anti_longti = 0.0;
      }
      if (initObj.hasOwnProperty('anti_press')) {
        this.anti_press = initObj.anti_press
      }
      else {
        this.anti_press = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type fc_to_oa
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [q0]
    bufferOffset = _serializer.float32(obj.q0, buffer, bufferOffset);
    // Serialize message field [q1]
    bufferOffset = _serializer.float32(obj.q1, buffer, bufferOffset);
    // Serialize message field [q2]
    bufferOffset = _serializer.float32(obj.q2, buffer, bufferOffset);
    // Serialize message field [q3]
    bufferOffset = _serializer.float32(obj.q3, buffer, bufferOffset);
    // Serialize message field [gyro_x]
    bufferOffset = _serializer.float32(obj.gyro_x, buffer, bufferOffset);
    // Serialize message field [gyro_y]
    bufferOffset = _serializer.float32(obj.gyro_y, buffer, bufferOffset);
    // Serialize message field [gyro_z]
    bufferOffset = _serializer.float32(obj.gyro_z, buffer, bufferOffset);
    // Serialize message field [acc_x]
    bufferOffset = _serializer.float32(obj.acc_x, buffer, bufferOffset);
    // Serialize message field [acc_y]
    bufferOffset = _serializer.float32(obj.acc_y, buffer, bufferOffset);
    // Serialize message field [acc_z]
    bufferOffset = _serializer.float32(obj.acc_z, buffer, bufferOffset);
    // Serialize message field [down_dist]
    bufferOffset = _serializer.float32(obj.down_dist, buffer, bufferOffset);
    // Serialize message field [top_dist]
    bufferOffset = _serializer.float32(obj.top_dist, buffer, bufferOffset);
    // Serialize message field [lati]
    bufferOffset = _serializer.float32(obj.lati, buffer, bufferOffset);
    // Serialize message field [longti]
    bufferOffset = _serializer.float32(obj.longti, buffer, bufferOffset);
    // Serialize message field [press]
    bufferOffset = _serializer.float32(obj.press, buffer, bufferOffset);
    // Serialize message field [vg_x]
    bufferOffset = _serializer.float32(obj.vg_x, buffer, bufferOffset);
    // Serialize message field [vg_y]
    bufferOffset = _serializer.float32(obj.vg_y, buffer, bufferOffset);
    // Serialize message field [vg_z]
    bufferOffset = _serializer.float32(obj.vg_z, buffer, bufferOffset);
    // Serialize message field [aileron]
    bufferOffset = _serializer.int16(obj.aileron, buffer, bufferOffset);
    // Serialize message field [elevator]
    bufferOffset = _serializer.int16(obj.elevator, buffer, bufferOffset);
    // Serialize message field [throttle]
    bufferOffset = _serializer.int16(obj.throttle, buffer, bufferOffset);
    // Serialize message field [rudder]
    bufferOffset = _serializer.int16(obj.rudder, buffer, bufferOffset);
    // Serialize message field [fc_timestamp_ms]
    bufferOffset = _serializer.uint32(obj.fc_timestamp_ms, buffer, bufferOffset);
    // Serialize message field [vo_flag]
    bufferOffset = _serializer.int16(obj.vo_flag, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float32(obj.roll, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float32(obj.pitch, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [anti_longti]
    bufferOffset = _serializer.float32(obj.anti_longti, buffer, bufferOffset);
    // Serialize message field [anti_press]
    bufferOffset = _serializer.float32(obj.anti_press, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type fc_to_oa
    let len;
    let data = new fc_to_oa(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [q0]
    data.q0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [q1]
    data.q1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [q2]
    data.q2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [q3]
    data.q3 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gyro_x]
    data.gyro_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gyro_y]
    data.gyro_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gyro_z]
    data.gyro_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [acc_x]
    data.acc_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [acc_y]
    data.acc_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [acc_z]
    data.acc_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [down_dist]
    data.down_dist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [top_dist]
    data.top_dist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lati]
    data.lati = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [longti]
    data.longti = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [press]
    data.press = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vg_x]
    data.vg_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vg_y]
    data.vg_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vg_z]
    data.vg_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [aileron]
    data.aileron = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [elevator]
    data.elevator = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [throttle]
    data.throttle = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [rudder]
    data.rudder = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [fc_timestamp_ms]
    data.fc_timestamp_ms = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [vo_flag]
    data.vo_flag = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [anti_longti]
    data.anti_longti = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [anti_press]
    data.anti_press = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 106;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/fc_to_oa';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f2d437dc8b62cfe7d33ce34dfc5bd422';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    float32 q0
    float32 q1
    float32 q2
    float32 q3
    float32 gyro_x
    float32 gyro_y
    float32 gyro_z
    float32 acc_x
    float32 acc_y
    float32 acc_z
    float32 down_dist
    float32 top_dist
    float32 lati
    float32 longti
    float32 press
    float32 vg_x
    float32 vg_y
    float32 vg_z
    int16 aileron
    int16 elevator
    int16 throttle
    int16 rudder
    uint32 fc_timestamp_ms
    int16 vo_flag
    float32 roll
    float32 pitch
    float32 yaw
    float32 anti_longti
    float32 anti_press
    
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
    const resolved = new fc_to_oa(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.q0 !== undefined) {
      resolved.q0 = msg.q0;
    }
    else {
      resolved.q0 = 0.0
    }

    if (msg.q1 !== undefined) {
      resolved.q1 = msg.q1;
    }
    else {
      resolved.q1 = 0.0
    }

    if (msg.q2 !== undefined) {
      resolved.q2 = msg.q2;
    }
    else {
      resolved.q2 = 0.0
    }

    if (msg.q3 !== undefined) {
      resolved.q3 = msg.q3;
    }
    else {
      resolved.q3 = 0.0
    }

    if (msg.gyro_x !== undefined) {
      resolved.gyro_x = msg.gyro_x;
    }
    else {
      resolved.gyro_x = 0.0
    }

    if (msg.gyro_y !== undefined) {
      resolved.gyro_y = msg.gyro_y;
    }
    else {
      resolved.gyro_y = 0.0
    }

    if (msg.gyro_z !== undefined) {
      resolved.gyro_z = msg.gyro_z;
    }
    else {
      resolved.gyro_z = 0.0
    }

    if (msg.acc_x !== undefined) {
      resolved.acc_x = msg.acc_x;
    }
    else {
      resolved.acc_x = 0.0
    }

    if (msg.acc_y !== undefined) {
      resolved.acc_y = msg.acc_y;
    }
    else {
      resolved.acc_y = 0.0
    }

    if (msg.acc_z !== undefined) {
      resolved.acc_z = msg.acc_z;
    }
    else {
      resolved.acc_z = 0.0
    }

    if (msg.down_dist !== undefined) {
      resolved.down_dist = msg.down_dist;
    }
    else {
      resolved.down_dist = 0.0
    }

    if (msg.top_dist !== undefined) {
      resolved.top_dist = msg.top_dist;
    }
    else {
      resolved.top_dist = 0.0
    }

    if (msg.lati !== undefined) {
      resolved.lati = msg.lati;
    }
    else {
      resolved.lati = 0.0
    }

    if (msg.longti !== undefined) {
      resolved.longti = msg.longti;
    }
    else {
      resolved.longti = 0.0
    }

    if (msg.press !== undefined) {
      resolved.press = msg.press;
    }
    else {
      resolved.press = 0.0
    }

    if (msg.vg_x !== undefined) {
      resolved.vg_x = msg.vg_x;
    }
    else {
      resolved.vg_x = 0.0
    }

    if (msg.vg_y !== undefined) {
      resolved.vg_y = msg.vg_y;
    }
    else {
      resolved.vg_y = 0.0
    }

    if (msg.vg_z !== undefined) {
      resolved.vg_z = msg.vg_z;
    }
    else {
      resolved.vg_z = 0.0
    }

    if (msg.aileron !== undefined) {
      resolved.aileron = msg.aileron;
    }
    else {
      resolved.aileron = 0
    }

    if (msg.elevator !== undefined) {
      resolved.elevator = msg.elevator;
    }
    else {
      resolved.elevator = 0
    }

    if (msg.throttle !== undefined) {
      resolved.throttle = msg.throttle;
    }
    else {
      resolved.throttle = 0
    }

    if (msg.rudder !== undefined) {
      resolved.rudder = msg.rudder;
    }
    else {
      resolved.rudder = 0
    }

    if (msg.fc_timestamp_ms !== undefined) {
      resolved.fc_timestamp_ms = msg.fc_timestamp_ms;
    }
    else {
      resolved.fc_timestamp_ms = 0
    }

    if (msg.vo_flag !== undefined) {
      resolved.vo_flag = msg.vo_flag;
    }
    else {
      resolved.vo_flag = 0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.anti_longti !== undefined) {
      resolved.anti_longti = msg.anti_longti;
    }
    else {
      resolved.anti_longti = 0.0
    }

    if (msg.anti_press !== undefined) {
      resolved.anti_press = msg.anti_press;
    }
    else {
      resolved.anti_press = 0.0
    }

    return resolved;
    }
};

module.exports = fc_to_oa;
