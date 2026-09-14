; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude oa_manager_debug.msg.html

(cl:defclass <oa_manager_debug> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (stereo_image_status_0_
    :reader stereo_image_status_0_
    :initarg :stereo_image_status_0_
    :type cl:fixnum
    :initform 0)
   (stereo_image_status_1_
    :reader stereo_image_status_1_
    :initarg :stereo_image_status_1_
    :type cl:fixnum
    :initform 0)
   (stereo_image_status_2_
    :reader stereo_image_status_2_
    :initarg :stereo_image_status_2_
    :type cl:fixnum
    :initform 0)
   (stereo_image_status_3_
    :reader stereo_image_status_3_
    :initarg :stereo_image_status_3_
    :type cl:fixnum
    :initform 0)
   (stereo_image_status_4_
    :reader stereo_image_status_4_
    :initarg :stereo_image_status_4_
    :type cl:fixnum
    :initform 0)
   (stereo_image_status_5_
    :reader stereo_image_status_5_
    :initarg :stereo_image_status_5_
    :type cl:fixnum
    :initform 0)
   (motor_active
    :reader motor_active
    :initarg :motor_active
    :type cl:fixnum
    :initform 0)
   (in_air
    :reader in_air
    :initarg :in_air
    :type cl:fixnum
    :initform 0)
   (fc_ctrl_mode
    :reader fc_ctrl_mode
    :initarg :fc_ctrl_mode
    :type cl:fixnum
    :initform 0)
   (wm232_front_flag
    :reader wm232_front_flag
    :initarg :wm232_front_flag
    :type cl:fixnum
    :initform 0)
   (tof_ground_filter_flag_0_
    :reader tof_ground_filter_flag_0_
    :initarg :tof_ground_filter_flag_0_
    :type cl:fixnum
    :initform 0)
   (tof_ground_filter_flag_1_
    :reader tof_ground_filter_flag_1_
    :initarg :tof_ground_filter_flag_1_
    :type cl:fixnum
    :initform 0)
   (tof_ground_filter_flag_2_
    :reader tof_ground_filter_flag_2_
    :initarg :tof_ground_filter_flag_2_
    :type cl:fixnum
    :initform 0)
   (tof_ground_filter_flag_3_
    :reader tof_ground_filter_flag_3_
    :initarg :tof_ground_filter_flag_3_
    :type cl:fixnum
    :initform 0)
   (tof_ground_filter_flag_4_
    :reader tof_ground_filter_flag_4_
    :initarg :tof_ground_filter_flag_4_
    :type cl:fixnum
    :initform 0)
   (tof_ground_filter_flag_5_
    :reader tof_ground_filter_flag_5_
    :initarg :tof_ground_filter_flag_5_
    :type cl:fixnum
    :initform 0)
   (radar_up_output_mm
    :reader radar_up_output_mm
    :initarg :radar_up_output_mm
    :type cl:fixnum
    :initform 0)
   (radar_down_output_mm
    :reader radar_down_output_mm
    :initarg :radar_down_output_mm
    :type cl:fixnum
    :initform 0)
   (radar_hon_disp_range_cfg_mm
    :reader radar_hon_disp_range_cfg_mm
    :initarg :radar_hon_disp_range_cfg_mm
    :type cl:fixnum
    :initform 0)
   (radar_down_disp_range_cfg_mm
    :reader radar_down_disp_range_cfg_mm
    :initarg :radar_down_disp_range_cfg_mm
    :type cl:fixnum
    :initform 0)
   (radar_up_disp_range_cfg_mm
    :reader radar_up_disp_range_cfg_mm
    :initarg :radar_up_disp_range_cfg_mm
    :type cl:fixnum
    :initform 0)
   (large_atti_no_oa
    :reader large_atti_no_oa
    :initarg :large_atti_no_oa
    :type cl:fixnum
    :initform 0)
   (oa_status_top_bar
    :reader oa_status_top_bar
    :initarg :oa_status_top_bar
    :type cl:fixnum
    :initform 0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (radar_graph_0_
    :reader radar_graph_0_
    :initarg :radar_graph_0_
    :type cl:integer
    :initform 0)
   (radar_graph_1_
    :reader radar_graph_1_
    :initarg :radar_graph_1_
    :type cl:integer
    :initform 0)
   (radar_graph_2_
    :reader radar_graph_2_
    :initarg :radar_graph_2_
    :type cl:integer
    :initform 0)
   (radar_graph_3_
    :reader radar_graph_3_
    :initarg :radar_graph_3_
    :type cl:integer
    :initform 0)
   (tof_radar_graph_data_mm_0_
    :reader tof_radar_graph_data_mm_0_
    :initarg :tof_radar_graph_data_mm_0_
    :type cl:fixnum
    :initform 0)
   (tof_radar_graph_data_mm_1_
    :reader tof_radar_graph_data_mm_1_
    :initarg :tof_radar_graph_data_mm_1_
    :type cl:fixnum
    :initform 0)
   (tof_radar_graph_data_mm_2_
    :reader tof_radar_graph_data_mm_2_
    :initarg :tof_radar_graph_data_mm_2_
    :type cl:fixnum
    :initform 0)
   (tof_radar_graph_data_mm_3_
    :reader tof_radar_graph_data_mm_3_
    :initarg :tof_radar_graph_data_mm_3_
    :type cl:fixnum
    :initform 0)
   (img_sts_ok_0_
    :reader img_sts_ok_0_
    :initarg :img_sts_ok_0_
    :type cl:fixnum
    :initform 0)
   (img_sts_ok_1_
    :reader img_sts_ok_1_
    :initarg :img_sts_ok_1_
    :type cl:fixnum
    :initform 0)
   (img_sts_ok_2_
    :reader img_sts_ok_2_
    :initarg :img_sts_ok_2_
    :type cl:fixnum
    :initform 0)
   (img_sts_ok_3_
    :reader img_sts_ok_3_
    :initarg :img_sts_ok_3_
    :type cl:fixnum
    :initform 0)
   (img_sts_ok_4_
    :reader img_sts_ok_4_
    :initarg :img_sts_ok_4_
    :type cl:fixnum
    :initform 0)
   (img_sts_ok_5_
    :reader img_sts_ok_5_
    :initarg :img_sts_ok_5_
    :type cl:fixnum
    :initform 0)
   (abnormal_dark_0_
    :reader abnormal_dark_0_
    :initarg :abnormal_dark_0_
    :type cl:fixnum
    :initform 0)
   (abnormal_dark_1_
    :reader abnormal_dark_1_
    :initarg :abnormal_dark_1_
    :type cl:fixnum
    :initform 0)
   (abnormal_dark_2_
    :reader abnormal_dark_2_
    :initarg :abnormal_dark_2_
    :type cl:fixnum
    :initform 0)
   (abnormal_dark_3_
    :reader abnormal_dark_3_
    :initarg :abnormal_dark_3_
    :type cl:fixnum
    :initform 0)
   (abnormal_dark_4_
    :reader abnormal_dark_4_
    :initarg :abnormal_dark_4_
    :type cl:fixnum
    :initform 0)
   (abnormal_dark_5_
    :reader abnormal_dark_5_
    :initarg :abnormal_dark_5_
    :type cl:fixnum
    :initform 0)
   (abnormal_bright_0_
    :reader abnormal_bright_0_
    :initarg :abnormal_bright_0_
    :type cl:fixnum
    :initform 0)
   (abnormal_bright_1_
    :reader abnormal_bright_1_
    :initarg :abnormal_bright_1_
    :type cl:fixnum
    :initform 0)
   (abnormal_bright_2_
    :reader abnormal_bright_2_
    :initarg :abnormal_bright_2_
    :type cl:fixnum
    :initform 0)
   (abnormal_bright_3_
    :reader abnormal_bright_3_
    :initarg :abnormal_bright_3_
    :type cl:fixnum
    :initform 0)
   (abnormal_bright_4_
    :reader abnormal_bright_4_
    :initarg :abnormal_bright_4_
    :type cl:fixnum
    :initform 0)
   (abnormal_bright_5_
    :reader abnormal_bright_5_
    :initarg :abnormal_bright_5_
    :type cl:fixnum
    :initform 0)
   (abnormal_similarity_0_
    :reader abnormal_similarity_0_
    :initarg :abnormal_similarity_0_
    :type cl:fixnum
    :initform 0)
   (abnormal_similarity_1_
    :reader abnormal_similarity_1_
    :initarg :abnormal_similarity_1_
    :type cl:fixnum
    :initform 0)
   (abnormal_similarity_2_
    :reader abnormal_similarity_2_
    :initarg :abnormal_similarity_2_
    :type cl:fixnum
    :initform 0)
   (abnormal_similarity_3_
    :reader abnormal_similarity_3_
    :initarg :abnormal_similarity_3_
    :type cl:fixnum
    :initform 0)
   (abnormal_similarity_4_
    :reader abnormal_similarity_4_
    :initarg :abnormal_similarity_4_
    :type cl:fixnum
    :initform 0)
   (abnormal_similarity_5_
    :reader abnormal_similarity_5_
    :initarg :abnormal_similarity_5_
    :type cl:fixnum
    :initform 0))
)

(cl:defclass oa_manager_debug (<oa_manager_debug>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <oa_manager_debug>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'oa_manager_debug)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<oa_manager_debug> is deprecated: use quadrotor_msgs-msg:oa_manager_debug instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'stereo_image_status_0_-val :lambda-list '(m))
(cl:defmethod stereo_image_status_0_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:stereo_image_status_0_-val is deprecated.  Use quadrotor_msgs-msg:stereo_image_status_0_ instead.")
  (stereo_image_status_0_ m))

(cl:ensure-generic-function 'stereo_image_status_1_-val :lambda-list '(m))
(cl:defmethod stereo_image_status_1_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:stereo_image_status_1_-val is deprecated.  Use quadrotor_msgs-msg:stereo_image_status_1_ instead.")
  (stereo_image_status_1_ m))

(cl:ensure-generic-function 'stereo_image_status_2_-val :lambda-list '(m))
(cl:defmethod stereo_image_status_2_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:stereo_image_status_2_-val is deprecated.  Use quadrotor_msgs-msg:stereo_image_status_2_ instead.")
  (stereo_image_status_2_ m))

(cl:ensure-generic-function 'stereo_image_status_3_-val :lambda-list '(m))
(cl:defmethod stereo_image_status_3_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:stereo_image_status_3_-val is deprecated.  Use quadrotor_msgs-msg:stereo_image_status_3_ instead.")
  (stereo_image_status_3_ m))

(cl:ensure-generic-function 'stereo_image_status_4_-val :lambda-list '(m))
(cl:defmethod stereo_image_status_4_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:stereo_image_status_4_-val is deprecated.  Use quadrotor_msgs-msg:stereo_image_status_4_ instead.")
  (stereo_image_status_4_ m))

(cl:ensure-generic-function 'stereo_image_status_5_-val :lambda-list '(m))
(cl:defmethod stereo_image_status_5_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:stereo_image_status_5_-val is deprecated.  Use quadrotor_msgs-msg:stereo_image_status_5_ instead.")
  (stereo_image_status_5_ m))

(cl:ensure-generic-function 'motor_active-val :lambda-list '(m))
(cl:defmethod motor_active-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:motor_active-val is deprecated.  Use quadrotor_msgs-msg:motor_active instead.")
  (motor_active m))

(cl:ensure-generic-function 'in_air-val :lambda-list '(m))
(cl:defmethod in_air-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:in_air-val is deprecated.  Use quadrotor_msgs-msg:in_air instead.")
  (in_air m))

(cl:ensure-generic-function 'fc_ctrl_mode-val :lambda-list '(m))
(cl:defmethod fc_ctrl_mode-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:fc_ctrl_mode-val is deprecated.  Use quadrotor_msgs-msg:fc_ctrl_mode instead.")
  (fc_ctrl_mode m))

(cl:ensure-generic-function 'wm232_front_flag-val :lambda-list '(m))
(cl:defmethod wm232_front_flag-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:wm232_front_flag-val is deprecated.  Use quadrotor_msgs-msg:wm232_front_flag instead.")
  (wm232_front_flag m))

(cl:ensure-generic-function 'tof_ground_filter_flag_0_-val :lambda-list '(m))
(cl:defmethod tof_ground_filter_flag_0_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:tof_ground_filter_flag_0_-val is deprecated.  Use quadrotor_msgs-msg:tof_ground_filter_flag_0_ instead.")
  (tof_ground_filter_flag_0_ m))

(cl:ensure-generic-function 'tof_ground_filter_flag_1_-val :lambda-list '(m))
(cl:defmethod tof_ground_filter_flag_1_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:tof_ground_filter_flag_1_-val is deprecated.  Use quadrotor_msgs-msg:tof_ground_filter_flag_1_ instead.")
  (tof_ground_filter_flag_1_ m))

(cl:ensure-generic-function 'tof_ground_filter_flag_2_-val :lambda-list '(m))
(cl:defmethod tof_ground_filter_flag_2_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:tof_ground_filter_flag_2_-val is deprecated.  Use quadrotor_msgs-msg:tof_ground_filter_flag_2_ instead.")
  (tof_ground_filter_flag_2_ m))

(cl:ensure-generic-function 'tof_ground_filter_flag_3_-val :lambda-list '(m))
(cl:defmethod tof_ground_filter_flag_3_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:tof_ground_filter_flag_3_-val is deprecated.  Use quadrotor_msgs-msg:tof_ground_filter_flag_3_ instead.")
  (tof_ground_filter_flag_3_ m))

(cl:ensure-generic-function 'tof_ground_filter_flag_4_-val :lambda-list '(m))
(cl:defmethod tof_ground_filter_flag_4_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:tof_ground_filter_flag_4_-val is deprecated.  Use quadrotor_msgs-msg:tof_ground_filter_flag_4_ instead.")
  (tof_ground_filter_flag_4_ m))

(cl:ensure-generic-function 'tof_ground_filter_flag_5_-val :lambda-list '(m))
(cl:defmethod tof_ground_filter_flag_5_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:tof_ground_filter_flag_5_-val is deprecated.  Use quadrotor_msgs-msg:tof_ground_filter_flag_5_ instead.")
  (tof_ground_filter_flag_5_ m))

(cl:ensure-generic-function 'radar_up_output_mm-val :lambda-list '(m))
(cl:defmethod radar_up_output_mm-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:radar_up_output_mm-val is deprecated.  Use quadrotor_msgs-msg:radar_up_output_mm instead.")
  (radar_up_output_mm m))

(cl:ensure-generic-function 'radar_down_output_mm-val :lambda-list '(m))
(cl:defmethod radar_down_output_mm-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:radar_down_output_mm-val is deprecated.  Use quadrotor_msgs-msg:radar_down_output_mm instead.")
  (radar_down_output_mm m))

(cl:ensure-generic-function 'radar_hon_disp_range_cfg_mm-val :lambda-list '(m))
(cl:defmethod radar_hon_disp_range_cfg_mm-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:radar_hon_disp_range_cfg_mm-val is deprecated.  Use quadrotor_msgs-msg:radar_hon_disp_range_cfg_mm instead.")
  (radar_hon_disp_range_cfg_mm m))

(cl:ensure-generic-function 'radar_down_disp_range_cfg_mm-val :lambda-list '(m))
(cl:defmethod radar_down_disp_range_cfg_mm-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:radar_down_disp_range_cfg_mm-val is deprecated.  Use quadrotor_msgs-msg:radar_down_disp_range_cfg_mm instead.")
  (radar_down_disp_range_cfg_mm m))

(cl:ensure-generic-function 'radar_up_disp_range_cfg_mm-val :lambda-list '(m))
(cl:defmethod radar_up_disp_range_cfg_mm-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:radar_up_disp_range_cfg_mm-val is deprecated.  Use quadrotor_msgs-msg:radar_up_disp_range_cfg_mm instead.")
  (radar_up_disp_range_cfg_mm m))

(cl:ensure-generic-function 'large_atti_no_oa-val :lambda-list '(m))
(cl:defmethod large_atti_no_oa-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:large_atti_no_oa-val is deprecated.  Use quadrotor_msgs-msg:large_atti_no_oa instead.")
  (large_atti_no_oa m))

(cl:ensure-generic-function 'oa_status_top_bar-val :lambda-list '(m))
(cl:defmethod oa_status_top_bar-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:oa_status_top_bar-val is deprecated.  Use quadrotor_msgs-msg:oa_status_top_bar instead.")
  (oa_status_top_bar m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:yaw-val is deprecated.  Use quadrotor_msgs-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:pitch-val is deprecated.  Use quadrotor_msgs-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:roll-val is deprecated.  Use quadrotor_msgs-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'radar_graph_0_-val :lambda-list '(m))
(cl:defmethod radar_graph_0_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:radar_graph_0_-val is deprecated.  Use quadrotor_msgs-msg:radar_graph_0_ instead.")
  (radar_graph_0_ m))

(cl:ensure-generic-function 'radar_graph_1_-val :lambda-list '(m))
(cl:defmethod radar_graph_1_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:radar_graph_1_-val is deprecated.  Use quadrotor_msgs-msg:radar_graph_1_ instead.")
  (radar_graph_1_ m))

(cl:ensure-generic-function 'radar_graph_2_-val :lambda-list '(m))
(cl:defmethod radar_graph_2_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:radar_graph_2_-val is deprecated.  Use quadrotor_msgs-msg:radar_graph_2_ instead.")
  (radar_graph_2_ m))

(cl:ensure-generic-function 'radar_graph_3_-val :lambda-list '(m))
(cl:defmethod radar_graph_3_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:radar_graph_3_-val is deprecated.  Use quadrotor_msgs-msg:radar_graph_3_ instead.")
  (radar_graph_3_ m))

(cl:ensure-generic-function 'tof_radar_graph_data_mm_0_-val :lambda-list '(m))
(cl:defmethod tof_radar_graph_data_mm_0_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:tof_radar_graph_data_mm_0_-val is deprecated.  Use quadrotor_msgs-msg:tof_radar_graph_data_mm_0_ instead.")
  (tof_radar_graph_data_mm_0_ m))

(cl:ensure-generic-function 'tof_radar_graph_data_mm_1_-val :lambda-list '(m))
(cl:defmethod tof_radar_graph_data_mm_1_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:tof_radar_graph_data_mm_1_-val is deprecated.  Use quadrotor_msgs-msg:tof_radar_graph_data_mm_1_ instead.")
  (tof_radar_graph_data_mm_1_ m))

(cl:ensure-generic-function 'tof_radar_graph_data_mm_2_-val :lambda-list '(m))
(cl:defmethod tof_radar_graph_data_mm_2_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:tof_radar_graph_data_mm_2_-val is deprecated.  Use quadrotor_msgs-msg:tof_radar_graph_data_mm_2_ instead.")
  (tof_radar_graph_data_mm_2_ m))

(cl:ensure-generic-function 'tof_radar_graph_data_mm_3_-val :lambda-list '(m))
(cl:defmethod tof_radar_graph_data_mm_3_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:tof_radar_graph_data_mm_3_-val is deprecated.  Use quadrotor_msgs-msg:tof_radar_graph_data_mm_3_ instead.")
  (tof_radar_graph_data_mm_3_ m))

(cl:ensure-generic-function 'img_sts_ok_0_-val :lambda-list '(m))
(cl:defmethod img_sts_ok_0_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:img_sts_ok_0_-val is deprecated.  Use quadrotor_msgs-msg:img_sts_ok_0_ instead.")
  (img_sts_ok_0_ m))

(cl:ensure-generic-function 'img_sts_ok_1_-val :lambda-list '(m))
(cl:defmethod img_sts_ok_1_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:img_sts_ok_1_-val is deprecated.  Use quadrotor_msgs-msg:img_sts_ok_1_ instead.")
  (img_sts_ok_1_ m))

(cl:ensure-generic-function 'img_sts_ok_2_-val :lambda-list '(m))
(cl:defmethod img_sts_ok_2_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:img_sts_ok_2_-val is deprecated.  Use quadrotor_msgs-msg:img_sts_ok_2_ instead.")
  (img_sts_ok_2_ m))

(cl:ensure-generic-function 'img_sts_ok_3_-val :lambda-list '(m))
(cl:defmethod img_sts_ok_3_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:img_sts_ok_3_-val is deprecated.  Use quadrotor_msgs-msg:img_sts_ok_3_ instead.")
  (img_sts_ok_3_ m))

(cl:ensure-generic-function 'img_sts_ok_4_-val :lambda-list '(m))
(cl:defmethod img_sts_ok_4_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:img_sts_ok_4_-val is deprecated.  Use quadrotor_msgs-msg:img_sts_ok_4_ instead.")
  (img_sts_ok_4_ m))

(cl:ensure-generic-function 'img_sts_ok_5_-val :lambda-list '(m))
(cl:defmethod img_sts_ok_5_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:img_sts_ok_5_-val is deprecated.  Use quadrotor_msgs-msg:img_sts_ok_5_ instead.")
  (img_sts_ok_5_ m))

(cl:ensure-generic-function 'abnormal_dark_0_-val :lambda-list '(m))
(cl:defmethod abnormal_dark_0_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_dark_0_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_dark_0_ instead.")
  (abnormal_dark_0_ m))

(cl:ensure-generic-function 'abnormal_dark_1_-val :lambda-list '(m))
(cl:defmethod abnormal_dark_1_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_dark_1_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_dark_1_ instead.")
  (abnormal_dark_1_ m))

(cl:ensure-generic-function 'abnormal_dark_2_-val :lambda-list '(m))
(cl:defmethod abnormal_dark_2_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_dark_2_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_dark_2_ instead.")
  (abnormal_dark_2_ m))

(cl:ensure-generic-function 'abnormal_dark_3_-val :lambda-list '(m))
(cl:defmethod abnormal_dark_3_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_dark_3_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_dark_3_ instead.")
  (abnormal_dark_3_ m))

(cl:ensure-generic-function 'abnormal_dark_4_-val :lambda-list '(m))
(cl:defmethod abnormal_dark_4_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_dark_4_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_dark_4_ instead.")
  (abnormal_dark_4_ m))

(cl:ensure-generic-function 'abnormal_dark_5_-val :lambda-list '(m))
(cl:defmethod abnormal_dark_5_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_dark_5_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_dark_5_ instead.")
  (abnormal_dark_5_ m))

(cl:ensure-generic-function 'abnormal_bright_0_-val :lambda-list '(m))
(cl:defmethod abnormal_bright_0_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_bright_0_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_bright_0_ instead.")
  (abnormal_bright_0_ m))

(cl:ensure-generic-function 'abnormal_bright_1_-val :lambda-list '(m))
(cl:defmethod abnormal_bright_1_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_bright_1_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_bright_1_ instead.")
  (abnormal_bright_1_ m))

(cl:ensure-generic-function 'abnormal_bright_2_-val :lambda-list '(m))
(cl:defmethod abnormal_bright_2_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_bright_2_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_bright_2_ instead.")
  (abnormal_bright_2_ m))

(cl:ensure-generic-function 'abnormal_bright_3_-val :lambda-list '(m))
(cl:defmethod abnormal_bright_3_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_bright_3_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_bright_3_ instead.")
  (abnormal_bright_3_ m))

(cl:ensure-generic-function 'abnormal_bright_4_-val :lambda-list '(m))
(cl:defmethod abnormal_bright_4_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_bright_4_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_bright_4_ instead.")
  (abnormal_bright_4_ m))

(cl:ensure-generic-function 'abnormal_bright_5_-val :lambda-list '(m))
(cl:defmethod abnormal_bright_5_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_bright_5_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_bright_5_ instead.")
  (abnormal_bright_5_ m))

(cl:ensure-generic-function 'abnormal_similarity_0_-val :lambda-list '(m))
(cl:defmethod abnormal_similarity_0_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_similarity_0_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_similarity_0_ instead.")
  (abnormal_similarity_0_ m))

(cl:ensure-generic-function 'abnormal_similarity_1_-val :lambda-list '(m))
(cl:defmethod abnormal_similarity_1_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_similarity_1_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_similarity_1_ instead.")
  (abnormal_similarity_1_ m))

(cl:ensure-generic-function 'abnormal_similarity_2_-val :lambda-list '(m))
(cl:defmethod abnormal_similarity_2_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_similarity_2_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_similarity_2_ instead.")
  (abnormal_similarity_2_ m))

(cl:ensure-generic-function 'abnormal_similarity_3_-val :lambda-list '(m))
(cl:defmethod abnormal_similarity_3_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_similarity_3_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_similarity_3_ instead.")
  (abnormal_similarity_3_ m))

(cl:ensure-generic-function 'abnormal_similarity_4_-val :lambda-list '(m))
(cl:defmethod abnormal_similarity_4_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_similarity_4_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_similarity_4_ instead.")
  (abnormal_similarity_4_ m))

(cl:ensure-generic-function 'abnormal_similarity_5_-val :lambda-list '(m))
(cl:defmethod abnormal_similarity_5_-val ((m <oa_manager_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:abnormal_similarity_5_-val is deprecated.  Use quadrotor_msgs-msg:abnormal_similarity_5_ instead.")
  (abnormal_similarity_5_ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <oa_manager_debug>) ostream)
  "Serializes a message object of type '<oa_manager_debug>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stereo_image_status_0_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stereo_image_status_1_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stereo_image_status_2_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stereo_image_status_3_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stereo_image_status_4_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stereo_image_status_5_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor_active)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'in_air)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fc_ctrl_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wm232_front_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_ground_filter_flag_0_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_ground_filter_flag_1_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_ground_filter_flag_2_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_ground_filter_flag_3_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_ground_filter_flag_4_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_ground_filter_flag_5_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radar_up_output_mm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'radar_up_output_mm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radar_down_output_mm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'radar_down_output_mm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radar_hon_disp_range_cfg_mm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'radar_hon_disp_range_cfg_mm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radar_down_disp_range_cfg_mm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'radar_down_disp_range_cfg_mm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radar_up_disp_range_cfg_mm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'radar_up_disp_range_cfg_mm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'large_atti_no_oa)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'oa_status_top_bar)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'oa_status_top_bar)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'radar_graph_0_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'radar_graph_1_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'radar_graph_2_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'radar_graph_3_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_radar_graph_data_mm_0_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tof_radar_graph_data_mm_0_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_radar_graph_data_mm_1_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tof_radar_graph_data_mm_1_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_radar_graph_data_mm_2_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tof_radar_graph_data_mm_2_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_radar_graph_data_mm_3_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tof_radar_graph_data_mm_3_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'img_sts_ok_0_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'img_sts_ok_1_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'img_sts_ok_2_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'img_sts_ok_3_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'img_sts_ok_4_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'img_sts_ok_5_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_dark_0_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_dark_1_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_dark_2_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_dark_3_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_dark_4_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_dark_5_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_bright_0_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_bright_1_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_bright_2_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_bright_3_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_bright_4_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_bright_5_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_similarity_0_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_similarity_1_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_similarity_2_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_similarity_3_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_similarity_4_)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_similarity_5_)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <oa_manager_debug>) istream)
  "Deserializes a message object of type '<oa_manager_debug>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stereo_image_status_0_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stereo_image_status_1_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stereo_image_status_2_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stereo_image_status_3_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stereo_image_status_4_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stereo_image_status_5_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor_active)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'in_air)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fc_ctrl_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wm232_front_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_ground_filter_flag_0_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_ground_filter_flag_1_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_ground_filter_flag_2_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_ground_filter_flag_3_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_ground_filter_flag_4_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_ground_filter_flag_5_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radar_up_output_mm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'radar_up_output_mm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radar_down_output_mm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'radar_down_output_mm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radar_hon_disp_range_cfg_mm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'radar_hon_disp_range_cfg_mm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radar_down_disp_range_cfg_mm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'radar_down_disp_range_cfg_mm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radar_up_disp_range_cfg_mm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'radar_up_disp_range_cfg_mm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'large_atti_no_oa)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'oa_status_top_bar)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'oa_status_top_bar)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'radar_graph_0_) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'radar_graph_1_) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'radar_graph_2_) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'radar_graph_3_) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_radar_graph_data_mm_0_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tof_radar_graph_data_mm_0_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_radar_graph_data_mm_1_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tof_radar_graph_data_mm_1_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_radar_graph_data_mm_2_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tof_radar_graph_data_mm_2_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tof_radar_graph_data_mm_3_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tof_radar_graph_data_mm_3_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'img_sts_ok_0_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'img_sts_ok_1_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'img_sts_ok_2_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'img_sts_ok_3_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'img_sts_ok_4_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'img_sts_ok_5_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_dark_0_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_dark_1_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_dark_2_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_dark_3_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_dark_4_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_dark_5_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_bright_0_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_bright_1_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_bright_2_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_bright_3_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_bright_4_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_bright_5_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_similarity_0_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_similarity_1_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_similarity_2_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_similarity_3_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_similarity_4_)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abnormal_similarity_5_)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<oa_manager_debug>)))
  "Returns string type for a message object of type '<oa_manager_debug>"
  "quadrotor_msgs/oa_manager_debug")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'oa_manager_debug)))
  "Returns string type for a message object of type 'oa_manager_debug"
  "quadrotor_msgs/oa_manager_debug")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<oa_manager_debug>)))
  "Returns md5sum for a message object of type '<oa_manager_debug>"
  "2fe2189ab02dae1ff093d243f6f55759")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'oa_manager_debug)))
  "Returns md5sum for a message object of type 'oa_manager_debug"
  "2fe2189ab02dae1ff093d243f6f55759")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<oa_manager_debug>)))
  "Returns full string definition for message of type '<oa_manager_debug>"
  (cl:format cl:nil "std_msgs/Header header~%uint8 stereo_image_status_0_~%uint8 stereo_image_status_1_~%uint8 stereo_image_status_2_~%uint8 stereo_image_status_3_~%uint8 stereo_image_status_4_~%uint8 stereo_image_status_5_~%uint8 motor_active~%uint8 in_air~%uint8 fc_ctrl_mode~%uint8 wm232_front_flag~%uint8 tof_ground_filter_flag_0_~%uint8 tof_ground_filter_flag_1_~%uint8 tof_ground_filter_flag_2_~%uint8 tof_ground_filter_flag_3_~%uint8 tof_ground_filter_flag_4_~%uint8 tof_ground_filter_flag_5_~%uint16 radar_up_output_mm~%uint16 radar_down_output_mm~%uint16 radar_hon_disp_range_cfg_mm~%uint16 radar_down_disp_range_cfg_mm~%uint16 radar_up_disp_range_cfg_mm~%uint8 large_atti_no_oa~%uint16 oa_status_top_bar~%float32 yaw~%float32 pitch~%float32 roll~%int32 radar_graph_0_~%int32 radar_graph_1_~%int32 radar_graph_2_~%int32 radar_graph_3_~%uint16 tof_radar_graph_data_mm_0_~%uint16 tof_radar_graph_data_mm_1_~%uint16 tof_radar_graph_data_mm_2_~%uint16 tof_radar_graph_data_mm_3_~%uint8 img_sts_ok_0_~%uint8 img_sts_ok_1_~%uint8 img_sts_ok_2_~%uint8 img_sts_ok_3_~%uint8 img_sts_ok_4_~%uint8 img_sts_ok_5_~%uint8 abnormal_dark_0_~%uint8 abnormal_dark_1_~%uint8 abnormal_dark_2_~%uint8 abnormal_dark_3_~%uint8 abnormal_dark_4_~%uint8 abnormal_dark_5_~%uint8 abnormal_bright_0_~%uint8 abnormal_bright_1_~%uint8 abnormal_bright_2_~%uint8 abnormal_bright_3_~%uint8 abnormal_bright_4_~%uint8 abnormal_bright_5_~%uint8 abnormal_similarity_0_~%uint8 abnormal_similarity_1_~%uint8 abnormal_similarity_2_~%uint8 abnormal_similarity_3_~%uint8 abnormal_similarity_4_~%uint8 abnormal_similarity_5_~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'oa_manager_debug)))
  "Returns full string definition for message of type 'oa_manager_debug"
  (cl:format cl:nil "std_msgs/Header header~%uint8 stereo_image_status_0_~%uint8 stereo_image_status_1_~%uint8 stereo_image_status_2_~%uint8 stereo_image_status_3_~%uint8 stereo_image_status_4_~%uint8 stereo_image_status_5_~%uint8 motor_active~%uint8 in_air~%uint8 fc_ctrl_mode~%uint8 wm232_front_flag~%uint8 tof_ground_filter_flag_0_~%uint8 tof_ground_filter_flag_1_~%uint8 tof_ground_filter_flag_2_~%uint8 tof_ground_filter_flag_3_~%uint8 tof_ground_filter_flag_4_~%uint8 tof_ground_filter_flag_5_~%uint16 radar_up_output_mm~%uint16 radar_down_output_mm~%uint16 radar_hon_disp_range_cfg_mm~%uint16 radar_down_disp_range_cfg_mm~%uint16 radar_up_disp_range_cfg_mm~%uint8 large_atti_no_oa~%uint16 oa_status_top_bar~%float32 yaw~%float32 pitch~%float32 roll~%int32 radar_graph_0_~%int32 radar_graph_1_~%int32 radar_graph_2_~%int32 radar_graph_3_~%uint16 tof_radar_graph_data_mm_0_~%uint16 tof_radar_graph_data_mm_1_~%uint16 tof_radar_graph_data_mm_2_~%uint16 tof_radar_graph_data_mm_3_~%uint8 img_sts_ok_0_~%uint8 img_sts_ok_1_~%uint8 img_sts_ok_2_~%uint8 img_sts_ok_3_~%uint8 img_sts_ok_4_~%uint8 img_sts_ok_5_~%uint8 abnormal_dark_0_~%uint8 abnormal_dark_1_~%uint8 abnormal_dark_2_~%uint8 abnormal_dark_3_~%uint8 abnormal_dark_4_~%uint8 abnormal_dark_5_~%uint8 abnormal_bright_0_~%uint8 abnormal_bright_1_~%uint8 abnormal_bright_2_~%uint8 abnormal_bright_3_~%uint8 abnormal_bright_4_~%uint8 abnormal_bright_5_~%uint8 abnormal_similarity_0_~%uint8 abnormal_similarity_1_~%uint8 abnormal_similarity_2_~%uint8 abnormal_similarity_3_~%uint8 abnormal_similarity_4_~%uint8 abnormal_similarity_5_~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <oa_manager_debug>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     2
     2
     2
     2
     2
     1
     2
     4
     4
     4
     4
     4
     4
     4
     2
     2
     2
     2
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <oa_manager_debug>))
  "Converts a ROS message object to a list"
  (cl:list 'oa_manager_debug
    (cl:cons ':header (header msg))
    (cl:cons ':stereo_image_status_0_ (stereo_image_status_0_ msg))
    (cl:cons ':stereo_image_status_1_ (stereo_image_status_1_ msg))
    (cl:cons ':stereo_image_status_2_ (stereo_image_status_2_ msg))
    (cl:cons ':stereo_image_status_3_ (stereo_image_status_3_ msg))
    (cl:cons ':stereo_image_status_4_ (stereo_image_status_4_ msg))
    (cl:cons ':stereo_image_status_5_ (stereo_image_status_5_ msg))
    (cl:cons ':motor_active (motor_active msg))
    (cl:cons ':in_air (in_air msg))
    (cl:cons ':fc_ctrl_mode (fc_ctrl_mode msg))
    (cl:cons ':wm232_front_flag (wm232_front_flag msg))
    (cl:cons ':tof_ground_filter_flag_0_ (tof_ground_filter_flag_0_ msg))
    (cl:cons ':tof_ground_filter_flag_1_ (tof_ground_filter_flag_1_ msg))
    (cl:cons ':tof_ground_filter_flag_2_ (tof_ground_filter_flag_2_ msg))
    (cl:cons ':tof_ground_filter_flag_3_ (tof_ground_filter_flag_3_ msg))
    (cl:cons ':tof_ground_filter_flag_4_ (tof_ground_filter_flag_4_ msg))
    (cl:cons ':tof_ground_filter_flag_5_ (tof_ground_filter_flag_5_ msg))
    (cl:cons ':radar_up_output_mm (radar_up_output_mm msg))
    (cl:cons ':radar_down_output_mm (radar_down_output_mm msg))
    (cl:cons ':radar_hon_disp_range_cfg_mm (radar_hon_disp_range_cfg_mm msg))
    (cl:cons ':radar_down_disp_range_cfg_mm (radar_down_disp_range_cfg_mm msg))
    (cl:cons ':radar_up_disp_range_cfg_mm (radar_up_disp_range_cfg_mm msg))
    (cl:cons ':large_atti_no_oa (large_atti_no_oa msg))
    (cl:cons ':oa_status_top_bar (oa_status_top_bar msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':radar_graph_0_ (radar_graph_0_ msg))
    (cl:cons ':radar_graph_1_ (radar_graph_1_ msg))
    (cl:cons ':radar_graph_2_ (radar_graph_2_ msg))
    (cl:cons ':radar_graph_3_ (radar_graph_3_ msg))
    (cl:cons ':tof_radar_graph_data_mm_0_ (tof_radar_graph_data_mm_0_ msg))
    (cl:cons ':tof_radar_graph_data_mm_1_ (tof_radar_graph_data_mm_1_ msg))
    (cl:cons ':tof_radar_graph_data_mm_2_ (tof_radar_graph_data_mm_2_ msg))
    (cl:cons ':tof_radar_graph_data_mm_3_ (tof_radar_graph_data_mm_3_ msg))
    (cl:cons ':img_sts_ok_0_ (img_sts_ok_0_ msg))
    (cl:cons ':img_sts_ok_1_ (img_sts_ok_1_ msg))
    (cl:cons ':img_sts_ok_2_ (img_sts_ok_2_ msg))
    (cl:cons ':img_sts_ok_3_ (img_sts_ok_3_ msg))
    (cl:cons ':img_sts_ok_4_ (img_sts_ok_4_ msg))
    (cl:cons ':img_sts_ok_5_ (img_sts_ok_5_ msg))
    (cl:cons ':abnormal_dark_0_ (abnormal_dark_0_ msg))
    (cl:cons ':abnormal_dark_1_ (abnormal_dark_1_ msg))
    (cl:cons ':abnormal_dark_2_ (abnormal_dark_2_ msg))
    (cl:cons ':abnormal_dark_3_ (abnormal_dark_3_ msg))
    (cl:cons ':abnormal_dark_4_ (abnormal_dark_4_ msg))
    (cl:cons ':abnormal_dark_5_ (abnormal_dark_5_ msg))
    (cl:cons ':abnormal_bright_0_ (abnormal_bright_0_ msg))
    (cl:cons ':abnormal_bright_1_ (abnormal_bright_1_ msg))
    (cl:cons ':abnormal_bright_2_ (abnormal_bright_2_ msg))
    (cl:cons ':abnormal_bright_3_ (abnormal_bright_3_ msg))
    (cl:cons ':abnormal_bright_4_ (abnormal_bright_4_ msg))
    (cl:cons ':abnormal_bright_5_ (abnormal_bright_5_ msg))
    (cl:cons ':abnormal_similarity_0_ (abnormal_similarity_0_ msg))
    (cl:cons ':abnormal_similarity_1_ (abnormal_similarity_1_ msg))
    (cl:cons ':abnormal_similarity_2_ (abnormal_similarity_2_ msg))
    (cl:cons ':abnormal_similarity_3_ (abnormal_similarity_3_ msg))
    (cl:cons ':abnormal_similarity_4_ (abnormal_similarity_4_ msg))
    (cl:cons ':abnormal_similarity_5_ (abnormal_similarity_5_ msg))
))
