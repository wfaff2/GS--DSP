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

class oa_manager_debug {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.stereo_image_status_0_ = null;
      this.stereo_image_status_1_ = null;
      this.stereo_image_status_2_ = null;
      this.stereo_image_status_3_ = null;
      this.stereo_image_status_4_ = null;
      this.stereo_image_status_5_ = null;
      this.motor_active = null;
      this.in_air = null;
      this.fc_ctrl_mode = null;
      this.wm232_front_flag = null;
      this.tof_ground_filter_flag_0_ = null;
      this.tof_ground_filter_flag_1_ = null;
      this.tof_ground_filter_flag_2_ = null;
      this.tof_ground_filter_flag_3_ = null;
      this.tof_ground_filter_flag_4_ = null;
      this.tof_ground_filter_flag_5_ = null;
      this.radar_up_output_mm = null;
      this.radar_down_output_mm = null;
      this.radar_hon_disp_range_cfg_mm = null;
      this.radar_down_disp_range_cfg_mm = null;
      this.radar_up_disp_range_cfg_mm = null;
      this.large_atti_no_oa = null;
      this.oa_status_top_bar = null;
      this.yaw = null;
      this.pitch = null;
      this.roll = null;
      this.radar_graph_0_ = null;
      this.radar_graph_1_ = null;
      this.radar_graph_2_ = null;
      this.radar_graph_3_ = null;
      this.tof_radar_graph_data_mm_0_ = null;
      this.tof_radar_graph_data_mm_1_ = null;
      this.tof_radar_graph_data_mm_2_ = null;
      this.tof_radar_graph_data_mm_3_ = null;
      this.img_sts_ok_0_ = null;
      this.img_sts_ok_1_ = null;
      this.img_sts_ok_2_ = null;
      this.img_sts_ok_3_ = null;
      this.img_sts_ok_4_ = null;
      this.img_sts_ok_5_ = null;
      this.abnormal_dark_0_ = null;
      this.abnormal_dark_1_ = null;
      this.abnormal_dark_2_ = null;
      this.abnormal_dark_3_ = null;
      this.abnormal_dark_4_ = null;
      this.abnormal_dark_5_ = null;
      this.abnormal_bright_0_ = null;
      this.abnormal_bright_1_ = null;
      this.abnormal_bright_2_ = null;
      this.abnormal_bright_3_ = null;
      this.abnormal_bright_4_ = null;
      this.abnormal_bright_5_ = null;
      this.abnormal_similarity_0_ = null;
      this.abnormal_similarity_1_ = null;
      this.abnormal_similarity_2_ = null;
      this.abnormal_similarity_3_ = null;
      this.abnormal_similarity_4_ = null;
      this.abnormal_similarity_5_ = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('stereo_image_status_0_')) {
        this.stereo_image_status_0_ = initObj.stereo_image_status_0_
      }
      else {
        this.stereo_image_status_0_ = 0;
      }
      if (initObj.hasOwnProperty('stereo_image_status_1_')) {
        this.stereo_image_status_1_ = initObj.stereo_image_status_1_
      }
      else {
        this.stereo_image_status_1_ = 0;
      }
      if (initObj.hasOwnProperty('stereo_image_status_2_')) {
        this.stereo_image_status_2_ = initObj.stereo_image_status_2_
      }
      else {
        this.stereo_image_status_2_ = 0;
      }
      if (initObj.hasOwnProperty('stereo_image_status_3_')) {
        this.stereo_image_status_3_ = initObj.stereo_image_status_3_
      }
      else {
        this.stereo_image_status_3_ = 0;
      }
      if (initObj.hasOwnProperty('stereo_image_status_4_')) {
        this.stereo_image_status_4_ = initObj.stereo_image_status_4_
      }
      else {
        this.stereo_image_status_4_ = 0;
      }
      if (initObj.hasOwnProperty('stereo_image_status_5_')) {
        this.stereo_image_status_5_ = initObj.stereo_image_status_5_
      }
      else {
        this.stereo_image_status_5_ = 0;
      }
      if (initObj.hasOwnProperty('motor_active')) {
        this.motor_active = initObj.motor_active
      }
      else {
        this.motor_active = 0;
      }
      if (initObj.hasOwnProperty('in_air')) {
        this.in_air = initObj.in_air
      }
      else {
        this.in_air = 0;
      }
      if (initObj.hasOwnProperty('fc_ctrl_mode')) {
        this.fc_ctrl_mode = initObj.fc_ctrl_mode
      }
      else {
        this.fc_ctrl_mode = 0;
      }
      if (initObj.hasOwnProperty('wm232_front_flag')) {
        this.wm232_front_flag = initObj.wm232_front_flag
      }
      else {
        this.wm232_front_flag = 0;
      }
      if (initObj.hasOwnProperty('tof_ground_filter_flag_0_')) {
        this.tof_ground_filter_flag_0_ = initObj.tof_ground_filter_flag_0_
      }
      else {
        this.tof_ground_filter_flag_0_ = 0;
      }
      if (initObj.hasOwnProperty('tof_ground_filter_flag_1_')) {
        this.tof_ground_filter_flag_1_ = initObj.tof_ground_filter_flag_1_
      }
      else {
        this.tof_ground_filter_flag_1_ = 0;
      }
      if (initObj.hasOwnProperty('tof_ground_filter_flag_2_')) {
        this.tof_ground_filter_flag_2_ = initObj.tof_ground_filter_flag_2_
      }
      else {
        this.tof_ground_filter_flag_2_ = 0;
      }
      if (initObj.hasOwnProperty('tof_ground_filter_flag_3_')) {
        this.tof_ground_filter_flag_3_ = initObj.tof_ground_filter_flag_3_
      }
      else {
        this.tof_ground_filter_flag_3_ = 0;
      }
      if (initObj.hasOwnProperty('tof_ground_filter_flag_4_')) {
        this.tof_ground_filter_flag_4_ = initObj.tof_ground_filter_flag_4_
      }
      else {
        this.tof_ground_filter_flag_4_ = 0;
      }
      if (initObj.hasOwnProperty('tof_ground_filter_flag_5_')) {
        this.tof_ground_filter_flag_5_ = initObj.tof_ground_filter_flag_5_
      }
      else {
        this.tof_ground_filter_flag_5_ = 0;
      }
      if (initObj.hasOwnProperty('radar_up_output_mm')) {
        this.radar_up_output_mm = initObj.radar_up_output_mm
      }
      else {
        this.radar_up_output_mm = 0;
      }
      if (initObj.hasOwnProperty('radar_down_output_mm')) {
        this.radar_down_output_mm = initObj.radar_down_output_mm
      }
      else {
        this.radar_down_output_mm = 0;
      }
      if (initObj.hasOwnProperty('radar_hon_disp_range_cfg_mm')) {
        this.radar_hon_disp_range_cfg_mm = initObj.radar_hon_disp_range_cfg_mm
      }
      else {
        this.radar_hon_disp_range_cfg_mm = 0;
      }
      if (initObj.hasOwnProperty('radar_down_disp_range_cfg_mm')) {
        this.radar_down_disp_range_cfg_mm = initObj.radar_down_disp_range_cfg_mm
      }
      else {
        this.radar_down_disp_range_cfg_mm = 0;
      }
      if (initObj.hasOwnProperty('radar_up_disp_range_cfg_mm')) {
        this.radar_up_disp_range_cfg_mm = initObj.radar_up_disp_range_cfg_mm
      }
      else {
        this.radar_up_disp_range_cfg_mm = 0;
      }
      if (initObj.hasOwnProperty('large_atti_no_oa')) {
        this.large_atti_no_oa = initObj.large_atti_no_oa
      }
      else {
        this.large_atti_no_oa = 0;
      }
      if (initObj.hasOwnProperty('oa_status_top_bar')) {
        this.oa_status_top_bar = initObj.oa_status_top_bar
      }
      else {
        this.oa_status_top_bar = 0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('radar_graph_0_')) {
        this.radar_graph_0_ = initObj.radar_graph_0_
      }
      else {
        this.radar_graph_0_ = 0;
      }
      if (initObj.hasOwnProperty('radar_graph_1_')) {
        this.radar_graph_1_ = initObj.radar_graph_1_
      }
      else {
        this.radar_graph_1_ = 0;
      }
      if (initObj.hasOwnProperty('radar_graph_2_')) {
        this.radar_graph_2_ = initObj.radar_graph_2_
      }
      else {
        this.radar_graph_2_ = 0;
      }
      if (initObj.hasOwnProperty('radar_graph_3_')) {
        this.radar_graph_3_ = initObj.radar_graph_3_
      }
      else {
        this.radar_graph_3_ = 0;
      }
      if (initObj.hasOwnProperty('tof_radar_graph_data_mm_0_')) {
        this.tof_radar_graph_data_mm_0_ = initObj.tof_radar_graph_data_mm_0_
      }
      else {
        this.tof_radar_graph_data_mm_0_ = 0;
      }
      if (initObj.hasOwnProperty('tof_radar_graph_data_mm_1_')) {
        this.tof_radar_graph_data_mm_1_ = initObj.tof_radar_graph_data_mm_1_
      }
      else {
        this.tof_radar_graph_data_mm_1_ = 0;
      }
      if (initObj.hasOwnProperty('tof_radar_graph_data_mm_2_')) {
        this.tof_radar_graph_data_mm_2_ = initObj.tof_radar_graph_data_mm_2_
      }
      else {
        this.tof_radar_graph_data_mm_2_ = 0;
      }
      if (initObj.hasOwnProperty('tof_radar_graph_data_mm_3_')) {
        this.tof_radar_graph_data_mm_3_ = initObj.tof_radar_graph_data_mm_3_
      }
      else {
        this.tof_radar_graph_data_mm_3_ = 0;
      }
      if (initObj.hasOwnProperty('img_sts_ok_0_')) {
        this.img_sts_ok_0_ = initObj.img_sts_ok_0_
      }
      else {
        this.img_sts_ok_0_ = 0;
      }
      if (initObj.hasOwnProperty('img_sts_ok_1_')) {
        this.img_sts_ok_1_ = initObj.img_sts_ok_1_
      }
      else {
        this.img_sts_ok_1_ = 0;
      }
      if (initObj.hasOwnProperty('img_sts_ok_2_')) {
        this.img_sts_ok_2_ = initObj.img_sts_ok_2_
      }
      else {
        this.img_sts_ok_2_ = 0;
      }
      if (initObj.hasOwnProperty('img_sts_ok_3_')) {
        this.img_sts_ok_3_ = initObj.img_sts_ok_3_
      }
      else {
        this.img_sts_ok_3_ = 0;
      }
      if (initObj.hasOwnProperty('img_sts_ok_4_')) {
        this.img_sts_ok_4_ = initObj.img_sts_ok_4_
      }
      else {
        this.img_sts_ok_4_ = 0;
      }
      if (initObj.hasOwnProperty('img_sts_ok_5_')) {
        this.img_sts_ok_5_ = initObj.img_sts_ok_5_
      }
      else {
        this.img_sts_ok_5_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_dark_0_')) {
        this.abnormal_dark_0_ = initObj.abnormal_dark_0_
      }
      else {
        this.abnormal_dark_0_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_dark_1_')) {
        this.abnormal_dark_1_ = initObj.abnormal_dark_1_
      }
      else {
        this.abnormal_dark_1_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_dark_2_')) {
        this.abnormal_dark_2_ = initObj.abnormal_dark_2_
      }
      else {
        this.abnormal_dark_2_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_dark_3_')) {
        this.abnormal_dark_3_ = initObj.abnormal_dark_3_
      }
      else {
        this.abnormal_dark_3_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_dark_4_')) {
        this.abnormal_dark_4_ = initObj.abnormal_dark_4_
      }
      else {
        this.abnormal_dark_4_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_dark_5_')) {
        this.abnormal_dark_5_ = initObj.abnormal_dark_5_
      }
      else {
        this.abnormal_dark_5_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_bright_0_')) {
        this.abnormal_bright_0_ = initObj.abnormal_bright_0_
      }
      else {
        this.abnormal_bright_0_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_bright_1_')) {
        this.abnormal_bright_1_ = initObj.abnormal_bright_1_
      }
      else {
        this.abnormal_bright_1_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_bright_2_')) {
        this.abnormal_bright_2_ = initObj.abnormal_bright_2_
      }
      else {
        this.abnormal_bright_2_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_bright_3_')) {
        this.abnormal_bright_3_ = initObj.abnormal_bright_3_
      }
      else {
        this.abnormal_bright_3_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_bright_4_')) {
        this.abnormal_bright_4_ = initObj.abnormal_bright_4_
      }
      else {
        this.abnormal_bright_4_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_bright_5_')) {
        this.abnormal_bright_5_ = initObj.abnormal_bright_5_
      }
      else {
        this.abnormal_bright_5_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_similarity_0_')) {
        this.abnormal_similarity_0_ = initObj.abnormal_similarity_0_
      }
      else {
        this.abnormal_similarity_0_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_similarity_1_')) {
        this.abnormal_similarity_1_ = initObj.abnormal_similarity_1_
      }
      else {
        this.abnormal_similarity_1_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_similarity_2_')) {
        this.abnormal_similarity_2_ = initObj.abnormal_similarity_2_
      }
      else {
        this.abnormal_similarity_2_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_similarity_3_')) {
        this.abnormal_similarity_3_ = initObj.abnormal_similarity_3_
      }
      else {
        this.abnormal_similarity_3_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_similarity_4_')) {
        this.abnormal_similarity_4_ = initObj.abnormal_similarity_4_
      }
      else {
        this.abnormal_similarity_4_ = 0;
      }
      if (initObj.hasOwnProperty('abnormal_similarity_5_')) {
        this.abnormal_similarity_5_ = initObj.abnormal_similarity_5_
      }
      else {
        this.abnormal_similarity_5_ = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type oa_manager_debug
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [stereo_image_status_0_]
    bufferOffset = _serializer.uint8(obj.stereo_image_status_0_, buffer, bufferOffset);
    // Serialize message field [stereo_image_status_1_]
    bufferOffset = _serializer.uint8(obj.stereo_image_status_1_, buffer, bufferOffset);
    // Serialize message field [stereo_image_status_2_]
    bufferOffset = _serializer.uint8(obj.stereo_image_status_2_, buffer, bufferOffset);
    // Serialize message field [stereo_image_status_3_]
    bufferOffset = _serializer.uint8(obj.stereo_image_status_3_, buffer, bufferOffset);
    // Serialize message field [stereo_image_status_4_]
    bufferOffset = _serializer.uint8(obj.stereo_image_status_4_, buffer, bufferOffset);
    // Serialize message field [stereo_image_status_5_]
    bufferOffset = _serializer.uint8(obj.stereo_image_status_5_, buffer, bufferOffset);
    // Serialize message field [motor_active]
    bufferOffset = _serializer.uint8(obj.motor_active, buffer, bufferOffset);
    // Serialize message field [in_air]
    bufferOffset = _serializer.uint8(obj.in_air, buffer, bufferOffset);
    // Serialize message field [fc_ctrl_mode]
    bufferOffset = _serializer.uint8(obj.fc_ctrl_mode, buffer, bufferOffset);
    // Serialize message field [wm232_front_flag]
    bufferOffset = _serializer.uint8(obj.wm232_front_flag, buffer, bufferOffset);
    // Serialize message field [tof_ground_filter_flag_0_]
    bufferOffset = _serializer.uint8(obj.tof_ground_filter_flag_0_, buffer, bufferOffset);
    // Serialize message field [tof_ground_filter_flag_1_]
    bufferOffset = _serializer.uint8(obj.tof_ground_filter_flag_1_, buffer, bufferOffset);
    // Serialize message field [tof_ground_filter_flag_2_]
    bufferOffset = _serializer.uint8(obj.tof_ground_filter_flag_2_, buffer, bufferOffset);
    // Serialize message field [tof_ground_filter_flag_3_]
    bufferOffset = _serializer.uint8(obj.tof_ground_filter_flag_3_, buffer, bufferOffset);
    // Serialize message field [tof_ground_filter_flag_4_]
    bufferOffset = _serializer.uint8(obj.tof_ground_filter_flag_4_, buffer, bufferOffset);
    // Serialize message field [tof_ground_filter_flag_5_]
    bufferOffset = _serializer.uint8(obj.tof_ground_filter_flag_5_, buffer, bufferOffset);
    // Serialize message field [radar_up_output_mm]
    bufferOffset = _serializer.uint16(obj.radar_up_output_mm, buffer, bufferOffset);
    // Serialize message field [radar_down_output_mm]
    bufferOffset = _serializer.uint16(obj.radar_down_output_mm, buffer, bufferOffset);
    // Serialize message field [radar_hon_disp_range_cfg_mm]
    bufferOffset = _serializer.uint16(obj.radar_hon_disp_range_cfg_mm, buffer, bufferOffset);
    // Serialize message field [radar_down_disp_range_cfg_mm]
    bufferOffset = _serializer.uint16(obj.radar_down_disp_range_cfg_mm, buffer, bufferOffset);
    // Serialize message field [radar_up_disp_range_cfg_mm]
    bufferOffset = _serializer.uint16(obj.radar_up_disp_range_cfg_mm, buffer, bufferOffset);
    // Serialize message field [large_atti_no_oa]
    bufferOffset = _serializer.uint8(obj.large_atti_no_oa, buffer, bufferOffset);
    // Serialize message field [oa_status_top_bar]
    bufferOffset = _serializer.uint16(obj.oa_status_top_bar, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float32(obj.pitch, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float32(obj.roll, buffer, bufferOffset);
    // Serialize message field [radar_graph_0_]
    bufferOffset = _serializer.int32(obj.radar_graph_0_, buffer, bufferOffset);
    // Serialize message field [radar_graph_1_]
    bufferOffset = _serializer.int32(obj.radar_graph_1_, buffer, bufferOffset);
    // Serialize message field [radar_graph_2_]
    bufferOffset = _serializer.int32(obj.radar_graph_2_, buffer, bufferOffset);
    // Serialize message field [radar_graph_3_]
    bufferOffset = _serializer.int32(obj.radar_graph_3_, buffer, bufferOffset);
    // Serialize message field [tof_radar_graph_data_mm_0_]
    bufferOffset = _serializer.uint16(obj.tof_radar_graph_data_mm_0_, buffer, bufferOffset);
    // Serialize message field [tof_radar_graph_data_mm_1_]
    bufferOffset = _serializer.uint16(obj.tof_radar_graph_data_mm_1_, buffer, bufferOffset);
    // Serialize message field [tof_radar_graph_data_mm_2_]
    bufferOffset = _serializer.uint16(obj.tof_radar_graph_data_mm_2_, buffer, bufferOffset);
    // Serialize message field [tof_radar_graph_data_mm_3_]
    bufferOffset = _serializer.uint16(obj.tof_radar_graph_data_mm_3_, buffer, bufferOffset);
    // Serialize message field [img_sts_ok_0_]
    bufferOffset = _serializer.uint8(obj.img_sts_ok_0_, buffer, bufferOffset);
    // Serialize message field [img_sts_ok_1_]
    bufferOffset = _serializer.uint8(obj.img_sts_ok_1_, buffer, bufferOffset);
    // Serialize message field [img_sts_ok_2_]
    bufferOffset = _serializer.uint8(obj.img_sts_ok_2_, buffer, bufferOffset);
    // Serialize message field [img_sts_ok_3_]
    bufferOffset = _serializer.uint8(obj.img_sts_ok_3_, buffer, bufferOffset);
    // Serialize message field [img_sts_ok_4_]
    bufferOffset = _serializer.uint8(obj.img_sts_ok_4_, buffer, bufferOffset);
    // Serialize message field [img_sts_ok_5_]
    bufferOffset = _serializer.uint8(obj.img_sts_ok_5_, buffer, bufferOffset);
    // Serialize message field [abnormal_dark_0_]
    bufferOffset = _serializer.uint8(obj.abnormal_dark_0_, buffer, bufferOffset);
    // Serialize message field [abnormal_dark_1_]
    bufferOffset = _serializer.uint8(obj.abnormal_dark_1_, buffer, bufferOffset);
    // Serialize message field [abnormal_dark_2_]
    bufferOffset = _serializer.uint8(obj.abnormal_dark_2_, buffer, bufferOffset);
    // Serialize message field [abnormal_dark_3_]
    bufferOffset = _serializer.uint8(obj.abnormal_dark_3_, buffer, bufferOffset);
    // Serialize message field [abnormal_dark_4_]
    bufferOffset = _serializer.uint8(obj.abnormal_dark_4_, buffer, bufferOffset);
    // Serialize message field [abnormal_dark_5_]
    bufferOffset = _serializer.uint8(obj.abnormal_dark_5_, buffer, bufferOffset);
    // Serialize message field [abnormal_bright_0_]
    bufferOffset = _serializer.uint8(obj.abnormal_bright_0_, buffer, bufferOffset);
    // Serialize message field [abnormal_bright_1_]
    bufferOffset = _serializer.uint8(obj.abnormal_bright_1_, buffer, bufferOffset);
    // Serialize message field [abnormal_bright_2_]
    bufferOffset = _serializer.uint8(obj.abnormal_bright_2_, buffer, bufferOffset);
    // Serialize message field [abnormal_bright_3_]
    bufferOffset = _serializer.uint8(obj.abnormal_bright_3_, buffer, bufferOffset);
    // Serialize message field [abnormal_bright_4_]
    bufferOffset = _serializer.uint8(obj.abnormal_bright_4_, buffer, bufferOffset);
    // Serialize message field [abnormal_bright_5_]
    bufferOffset = _serializer.uint8(obj.abnormal_bright_5_, buffer, bufferOffset);
    // Serialize message field [abnormal_similarity_0_]
    bufferOffset = _serializer.uint8(obj.abnormal_similarity_0_, buffer, bufferOffset);
    // Serialize message field [abnormal_similarity_1_]
    bufferOffset = _serializer.uint8(obj.abnormal_similarity_1_, buffer, bufferOffset);
    // Serialize message field [abnormal_similarity_2_]
    bufferOffset = _serializer.uint8(obj.abnormal_similarity_2_, buffer, bufferOffset);
    // Serialize message field [abnormal_similarity_3_]
    bufferOffset = _serializer.uint8(obj.abnormal_similarity_3_, buffer, bufferOffset);
    // Serialize message field [abnormal_similarity_4_]
    bufferOffset = _serializer.uint8(obj.abnormal_similarity_4_, buffer, bufferOffset);
    // Serialize message field [abnormal_similarity_5_]
    bufferOffset = _serializer.uint8(obj.abnormal_similarity_5_, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type oa_manager_debug
    let len;
    let data = new oa_manager_debug(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [stereo_image_status_0_]
    data.stereo_image_status_0_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [stereo_image_status_1_]
    data.stereo_image_status_1_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [stereo_image_status_2_]
    data.stereo_image_status_2_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [stereo_image_status_3_]
    data.stereo_image_status_3_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [stereo_image_status_4_]
    data.stereo_image_status_4_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [stereo_image_status_5_]
    data.stereo_image_status_5_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [motor_active]
    data.motor_active = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [in_air]
    data.in_air = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [fc_ctrl_mode]
    data.fc_ctrl_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [wm232_front_flag]
    data.wm232_front_flag = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [tof_ground_filter_flag_0_]
    data.tof_ground_filter_flag_0_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [tof_ground_filter_flag_1_]
    data.tof_ground_filter_flag_1_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [tof_ground_filter_flag_2_]
    data.tof_ground_filter_flag_2_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [tof_ground_filter_flag_3_]
    data.tof_ground_filter_flag_3_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [tof_ground_filter_flag_4_]
    data.tof_ground_filter_flag_4_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [tof_ground_filter_flag_5_]
    data.tof_ground_filter_flag_5_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [radar_up_output_mm]
    data.radar_up_output_mm = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [radar_down_output_mm]
    data.radar_down_output_mm = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [radar_hon_disp_range_cfg_mm]
    data.radar_hon_disp_range_cfg_mm = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [radar_down_disp_range_cfg_mm]
    data.radar_down_disp_range_cfg_mm = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [radar_up_disp_range_cfg_mm]
    data.radar_up_disp_range_cfg_mm = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [large_atti_no_oa]
    data.large_atti_no_oa = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [oa_status_top_bar]
    data.oa_status_top_bar = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [radar_graph_0_]
    data.radar_graph_0_ = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [radar_graph_1_]
    data.radar_graph_1_ = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [radar_graph_2_]
    data.radar_graph_2_ = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [radar_graph_3_]
    data.radar_graph_3_ = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [tof_radar_graph_data_mm_0_]
    data.tof_radar_graph_data_mm_0_ = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [tof_radar_graph_data_mm_1_]
    data.tof_radar_graph_data_mm_1_ = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [tof_radar_graph_data_mm_2_]
    data.tof_radar_graph_data_mm_2_ = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [tof_radar_graph_data_mm_3_]
    data.tof_radar_graph_data_mm_3_ = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [img_sts_ok_0_]
    data.img_sts_ok_0_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [img_sts_ok_1_]
    data.img_sts_ok_1_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [img_sts_ok_2_]
    data.img_sts_ok_2_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [img_sts_ok_3_]
    data.img_sts_ok_3_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [img_sts_ok_4_]
    data.img_sts_ok_4_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [img_sts_ok_5_]
    data.img_sts_ok_5_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_dark_0_]
    data.abnormal_dark_0_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_dark_1_]
    data.abnormal_dark_1_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_dark_2_]
    data.abnormal_dark_2_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_dark_3_]
    data.abnormal_dark_3_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_dark_4_]
    data.abnormal_dark_4_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_dark_5_]
    data.abnormal_dark_5_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_bright_0_]
    data.abnormal_bright_0_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_bright_1_]
    data.abnormal_bright_1_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_bright_2_]
    data.abnormal_bright_2_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_bright_3_]
    data.abnormal_bright_3_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_bright_4_]
    data.abnormal_bright_4_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_bright_5_]
    data.abnormal_bright_5_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_similarity_0_]
    data.abnormal_similarity_0_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_similarity_1_]
    data.abnormal_similarity_1_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_similarity_2_]
    data.abnormal_similarity_2_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_similarity_3_]
    data.abnormal_similarity_3_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_similarity_4_]
    data.abnormal_similarity_4_ = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [abnormal_similarity_5_]
    data.abnormal_similarity_5_ = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 89;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/oa_manager_debug';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2fe2189ab02dae1ff093d243f6f55759';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    uint8 stereo_image_status_0_
    uint8 stereo_image_status_1_
    uint8 stereo_image_status_2_
    uint8 stereo_image_status_3_
    uint8 stereo_image_status_4_
    uint8 stereo_image_status_5_
    uint8 motor_active
    uint8 in_air
    uint8 fc_ctrl_mode
    uint8 wm232_front_flag
    uint8 tof_ground_filter_flag_0_
    uint8 tof_ground_filter_flag_1_
    uint8 tof_ground_filter_flag_2_
    uint8 tof_ground_filter_flag_3_
    uint8 tof_ground_filter_flag_4_
    uint8 tof_ground_filter_flag_5_
    uint16 radar_up_output_mm
    uint16 radar_down_output_mm
    uint16 radar_hon_disp_range_cfg_mm
    uint16 radar_down_disp_range_cfg_mm
    uint16 radar_up_disp_range_cfg_mm
    uint8 large_atti_no_oa
    uint16 oa_status_top_bar
    float32 yaw
    float32 pitch
    float32 roll
    int32 radar_graph_0_
    int32 radar_graph_1_
    int32 radar_graph_2_
    int32 radar_graph_3_
    uint16 tof_radar_graph_data_mm_0_
    uint16 tof_radar_graph_data_mm_1_
    uint16 tof_radar_graph_data_mm_2_
    uint16 tof_radar_graph_data_mm_3_
    uint8 img_sts_ok_0_
    uint8 img_sts_ok_1_
    uint8 img_sts_ok_2_
    uint8 img_sts_ok_3_
    uint8 img_sts_ok_4_
    uint8 img_sts_ok_5_
    uint8 abnormal_dark_0_
    uint8 abnormal_dark_1_
    uint8 abnormal_dark_2_
    uint8 abnormal_dark_3_
    uint8 abnormal_dark_4_
    uint8 abnormal_dark_5_
    uint8 abnormal_bright_0_
    uint8 abnormal_bright_1_
    uint8 abnormal_bright_2_
    uint8 abnormal_bright_3_
    uint8 abnormal_bright_4_
    uint8 abnormal_bright_5_
    uint8 abnormal_similarity_0_
    uint8 abnormal_similarity_1_
    uint8 abnormal_similarity_2_
    uint8 abnormal_similarity_3_
    uint8 abnormal_similarity_4_
    uint8 abnormal_similarity_5_
    
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
    const resolved = new oa_manager_debug(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.stereo_image_status_0_ !== undefined) {
      resolved.stereo_image_status_0_ = msg.stereo_image_status_0_;
    }
    else {
      resolved.stereo_image_status_0_ = 0
    }

    if (msg.stereo_image_status_1_ !== undefined) {
      resolved.stereo_image_status_1_ = msg.stereo_image_status_1_;
    }
    else {
      resolved.stereo_image_status_1_ = 0
    }

    if (msg.stereo_image_status_2_ !== undefined) {
      resolved.stereo_image_status_2_ = msg.stereo_image_status_2_;
    }
    else {
      resolved.stereo_image_status_2_ = 0
    }

    if (msg.stereo_image_status_3_ !== undefined) {
      resolved.stereo_image_status_3_ = msg.stereo_image_status_3_;
    }
    else {
      resolved.stereo_image_status_3_ = 0
    }

    if (msg.stereo_image_status_4_ !== undefined) {
      resolved.stereo_image_status_4_ = msg.stereo_image_status_4_;
    }
    else {
      resolved.stereo_image_status_4_ = 0
    }

    if (msg.stereo_image_status_5_ !== undefined) {
      resolved.stereo_image_status_5_ = msg.stereo_image_status_5_;
    }
    else {
      resolved.stereo_image_status_5_ = 0
    }

    if (msg.motor_active !== undefined) {
      resolved.motor_active = msg.motor_active;
    }
    else {
      resolved.motor_active = 0
    }

    if (msg.in_air !== undefined) {
      resolved.in_air = msg.in_air;
    }
    else {
      resolved.in_air = 0
    }

    if (msg.fc_ctrl_mode !== undefined) {
      resolved.fc_ctrl_mode = msg.fc_ctrl_mode;
    }
    else {
      resolved.fc_ctrl_mode = 0
    }

    if (msg.wm232_front_flag !== undefined) {
      resolved.wm232_front_flag = msg.wm232_front_flag;
    }
    else {
      resolved.wm232_front_flag = 0
    }

    if (msg.tof_ground_filter_flag_0_ !== undefined) {
      resolved.tof_ground_filter_flag_0_ = msg.tof_ground_filter_flag_0_;
    }
    else {
      resolved.tof_ground_filter_flag_0_ = 0
    }

    if (msg.tof_ground_filter_flag_1_ !== undefined) {
      resolved.tof_ground_filter_flag_1_ = msg.tof_ground_filter_flag_1_;
    }
    else {
      resolved.tof_ground_filter_flag_1_ = 0
    }

    if (msg.tof_ground_filter_flag_2_ !== undefined) {
      resolved.tof_ground_filter_flag_2_ = msg.tof_ground_filter_flag_2_;
    }
    else {
      resolved.tof_ground_filter_flag_2_ = 0
    }

    if (msg.tof_ground_filter_flag_3_ !== undefined) {
      resolved.tof_ground_filter_flag_3_ = msg.tof_ground_filter_flag_3_;
    }
    else {
      resolved.tof_ground_filter_flag_3_ = 0
    }

    if (msg.tof_ground_filter_flag_4_ !== undefined) {
      resolved.tof_ground_filter_flag_4_ = msg.tof_ground_filter_flag_4_;
    }
    else {
      resolved.tof_ground_filter_flag_4_ = 0
    }

    if (msg.tof_ground_filter_flag_5_ !== undefined) {
      resolved.tof_ground_filter_flag_5_ = msg.tof_ground_filter_flag_5_;
    }
    else {
      resolved.tof_ground_filter_flag_5_ = 0
    }

    if (msg.radar_up_output_mm !== undefined) {
      resolved.radar_up_output_mm = msg.radar_up_output_mm;
    }
    else {
      resolved.radar_up_output_mm = 0
    }

    if (msg.radar_down_output_mm !== undefined) {
      resolved.radar_down_output_mm = msg.radar_down_output_mm;
    }
    else {
      resolved.radar_down_output_mm = 0
    }

    if (msg.radar_hon_disp_range_cfg_mm !== undefined) {
      resolved.radar_hon_disp_range_cfg_mm = msg.radar_hon_disp_range_cfg_mm;
    }
    else {
      resolved.radar_hon_disp_range_cfg_mm = 0
    }

    if (msg.radar_down_disp_range_cfg_mm !== undefined) {
      resolved.radar_down_disp_range_cfg_mm = msg.radar_down_disp_range_cfg_mm;
    }
    else {
      resolved.radar_down_disp_range_cfg_mm = 0
    }

    if (msg.radar_up_disp_range_cfg_mm !== undefined) {
      resolved.radar_up_disp_range_cfg_mm = msg.radar_up_disp_range_cfg_mm;
    }
    else {
      resolved.radar_up_disp_range_cfg_mm = 0
    }

    if (msg.large_atti_no_oa !== undefined) {
      resolved.large_atti_no_oa = msg.large_atti_no_oa;
    }
    else {
      resolved.large_atti_no_oa = 0
    }

    if (msg.oa_status_top_bar !== undefined) {
      resolved.oa_status_top_bar = msg.oa_status_top_bar;
    }
    else {
      resolved.oa_status_top_bar = 0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.radar_graph_0_ !== undefined) {
      resolved.radar_graph_0_ = msg.radar_graph_0_;
    }
    else {
      resolved.radar_graph_0_ = 0
    }

    if (msg.radar_graph_1_ !== undefined) {
      resolved.radar_graph_1_ = msg.radar_graph_1_;
    }
    else {
      resolved.radar_graph_1_ = 0
    }

    if (msg.radar_graph_2_ !== undefined) {
      resolved.radar_graph_2_ = msg.radar_graph_2_;
    }
    else {
      resolved.radar_graph_2_ = 0
    }

    if (msg.radar_graph_3_ !== undefined) {
      resolved.radar_graph_3_ = msg.radar_graph_3_;
    }
    else {
      resolved.radar_graph_3_ = 0
    }

    if (msg.tof_radar_graph_data_mm_0_ !== undefined) {
      resolved.tof_radar_graph_data_mm_0_ = msg.tof_radar_graph_data_mm_0_;
    }
    else {
      resolved.tof_radar_graph_data_mm_0_ = 0
    }

    if (msg.tof_radar_graph_data_mm_1_ !== undefined) {
      resolved.tof_radar_graph_data_mm_1_ = msg.tof_radar_graph_data_mm_1_;
    }
    else {
      resolved.tof_radar_graph_data_mm_1_ = 0
    }

    if (msg.tof_radar_graph_data_mm_2_ !== undefined) {
      resolved.tof_radar_graph_data_mm_2_ = msg.tof_radar_graph_data_mm_2_;
    }
    else {
      resolved.tof_radar_graph_data_mm_2_ = 0
    }

    if (msg.tof_radar_graph_data_mm_3_ !== undefined) {
      resolved.tof_radar_graph_data_mm_3_ = msg.tof_radar_graph_data_mm_3_;
    }
    else {
      resolved.tof_radar_graph_data_mm_3_ = 0
    }

    if (msg.img_sts_ok_0_ !== undefined) {
      resolved.img_sts_ok_0_ = msg.img_sts_ok_0_;
    }
    else {
      resolved.img_sts_ok_0_ = 0
    }

    if (msg.img_sts_ok_1_ !== undefined) {
      resolved.img_sts_ok_1_ = msg.img_sts_ok_1_;
    }
    else {
      resolved.img_sts_ok_1_ = 0
    }

    if (msg.img_sts_ok_2_ !== undefined) {
      resolved.img_sts_ok_2_ = msg.img_sts_ok_2_;
    }
    else {
      resolved.img_sts_ok_2_ = 0
    }

    if (msg.img_sts_ok_3_ !== undefined) {
      resolved.img_sts_ok_3_ = msg.img_sts_ok_3_;
    }
    else {
      resolved.img_sts_ok_3_ = 0
    }

    if (msg.img_sts_ok_4_ !== undefined) {
      resolved.img_sts_ok_4_ = msg.img_sts_ok_4_;
    }
    else {
      resolved.img_sts_ok_4_ = 0
    }

    if (msg.img_sts_ok_5_ !== undefined) {
      resolved.img_sts_ok_5_ = msg.img_sts_ok_5_;
    }
    else {
      resolved.img_sts_ok_5_ = 0
    }

    if (msg.abnormal_dark_0_ !== undefined) {
      resolved.abnormal_dark_0_ = msg.abnormal_dark_0_;
    }
    else {
      resolved.abnormal_dark_0_ = 0
    }

    if (msg.abnormal_dark_1_ !== undefined) {
      resolved.abnormal_dark_1_ = msg.abnormal_dark_1_;
    }
    else {
      resolved.abnormal_dark_1_ = 0
    }

    if (msg.abnormal_dark_2_ !== undefined) {
      resolved.abnormal_dark_2_ = msg.abnormal_dark_2_;
    }
    else {
      resolved.abnormal_dark_2_ = 0
    }

    if (msg.abnormal_dark_3_ !== undefined) {
      resolved.abnormal_dark_3_ = msg.abnormal_dark_3_;
    }
    else {
      resolved.abnormal_dark_3_ = 0
    }

    if (msg.abnormal_dark_4_ !== undefined) {
      resolved.abnormal_dark_4_ = msg.abnormal_dark_4_;
    }
    else {
      resolved.abnormal_dark_4_ = 0
    }

    if (msg.abnormal_dark_5_ !== undefined) {
      resolved.abnormal_dark_5_ = msg.abnormal_dark_5_;
    }
    else {
      resolved.abnormal_dark_5_ = 0
    }

    if (msg.abnormal_bright_0_ !== undefined) {
      resolved.abnormal_bright_0_ = msg.abnormal_bright_0_;
    }
    else {
      resolved.abnormal_bright_0_ = 0
    }

    if (msg.abnormal_bright_1_ !== undefined) {
      resolved.abnormal_bright_1_ = msg.abnormal_bright_1_;
    }
    else {
      resolved.abnormal_bright_1_ = 0
    }

    if (msg.abnormal_bright_2_ !== undefined) {
      resolved.abnormal_bright_2_ = msg.abnormal_bright_2_;
    }
    else {
      resolved.abnormal_bright_2_ = 0
    }

    if (msg.abnormal_bright_3_ !== undefined) {
      resolved.abnormal_bright_3_ = msg.abnormal_bright_3_;
    }
    else {
      resolved.abnormal_bright_3_ = 0
    }

    if (msg.abnormal_bright_4_ !== undefined) {
      resolved.abnormal_bright_4_ = msg.abnormal_bright_4_;
    }
    else {
      resolved.abnormal_bright_4_ = 0
    }

    if (msg.abnormal_bright_5_ !== undefined) {
      resolved.abnormal_bright_5_ = msg.abnormal_bright_5_;
    }
    else {
      resolved.abnormal_bright_5_ = 0
    }

    if (msg.abnormal_similarity_0_ !== undefined) {
      resolved.abnormal_similarity_0_ = msg.abnormal_similarity_0_;
    }
    else {
      resolved.abnormal_similarity_0_ = 0
    }

    if (msg.abnormal_similarity_1_ !== undefined) {
      resolved.abnormal_similarity_1_ = msg.abnormal_similarity_1_;
    }
    else {
      resolved.abnormal_similarity_1_ = 0
    }

    if (msg.abnormal_similarity_2_ !== undefined) {
      resolved.abnormal_similarity_2_ = msg.abnormal_similarity_2_;
    }
    else {
      resolved.abnormal_similarity_2_ = 0
    }

    if (msg.abnormal_similarity_3_ !== undefined) {
      resolved.abnormal_similarity_3_ = msg.abnormal_similarity_3_;
    }
    else {
      resolved.abnormal_similarity_3_ = 0
    }

    if (msg.abnormal_similarity_4_ !== undefined) {
      resolved.abnormal_similarity_4_ = msg.abnormal_similarity_4_;
    }
    else {
      resolved.abnormal_similarity_4_ = 0
    }

    if (msg.abnormal_similarity_5_ !== undefined) {
      resolved.abnormal_similarity_5_ = msg.abnormal_similarity_5_;
    }
    else {
      resolved.abnormal_similarity_5_ = 0
    }

    return resolved;
    }
};

module.exports = oa_manager_debug;
