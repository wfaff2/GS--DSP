;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::oa_manager_debug)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'oa_manager_debug (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::OA_MANAGER_DEBUG")
  (make-package "QUADROTOR_MSGS::OA_MANAGER_DEBUG"))

(in-package "ROS")
;;//! \htmlinclude oa_manager_debug.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::oa_manager_debug
  :super ros::object
  :slots (_header _stereo_image_status_0_ _stereo_image_status_1_ _stereo_image_status_2_ _stereo_image_status_3_ _stereo_image_status_4_ _stereo_image_status_5_ _motor_active _in_air _fc_ctrl_mode _wm232_front_flag _tof_ground_filter_flag_0_ _tof_ground_filter_flag_1_ _tof_ground_filter_flag_2_ _tof_ground_filter_flag_3_ _tof_ground_filter_flag_4_ _tof_ground_filter_flag_5_ _radar_up_output_mm _radar_down_output_mm _radar_hon_disp_range_cfg_mm _radar_down_disp_range_cfg_mm _radar_up_disp_range_cfg_mm _large_atti_no_oa _oa_status_top_bar _yaw _pitch _roll _radar_graph_0_ _radar_graph_1_ _radar_graph_2_ _radar_graph_3_ _tof_radar_graph_data_mm_0_ _tof_radar_graph_data_mm_1_ _tof_radar_graph_data_mm_2_ _tof_radar_graph_data_mm_3_ _img_sts_ok_0_ _img_sts_ok_1_ _img_sts_ok_2_ _img_sts_ok_3_ _img_sts_ok_4_ _img_sts_ok_5_ _abnormal_dark_0_ _abnormal_dark_1_ _abnormal_dark_2_ _abnormal_dark_3_ _abnormal_dark_4_ _abnormal_dark_5_ _abnormal_bright_0_ _abnormal_bright_1_ _abnormal_bright_2_ _abnormal_bright_3_ _abnormal_bright_4_ _abnormal_bright_5_ _abnormal_similarity_0_ _abnormal_similarity_1_ _abnormal_similarity_2_ _abnormal_similarity_3_ _abnormal_similarity_4_ _abnormal_similarity_5_ ))

(defmethod quadrotor_msgs::oa_manager_debug
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:stereo_image_status_0_ __stereo_image_status_0_) 0)
    ((:stereo_image_status_1_ __stereo_image_status_1_) 0)
    ((:stereo_image_status_2_ __stereo_image_status_2_) 0)
    ((:stereo_image_status_3_ __stereo_image_status_3_) 0)
    ((:stereo_image_status_4_ __stereo_image_status_4_) 0)
    ((:stereo_image_status_5_ __stereo_image_status_5_) 0)
    ((:motor_active __motor_active) 0)
    ((:in_air __in_air) 0)
    ((:fc_ctrl_mode __fc_ctrl_mode) 0)
    ((:wm232_front_flag __wm232_front_flag) 0)
    ((:tof_ground_filter_flag_0_ __tof_ground_filter_flag_0_) 0)
    ((:tof_ground_filter_flag_1_ __tof_ground_filter_flag_1_) 0)
    ((:tof_ground_filter_flag_2_ __tof_ground_filter_flag_2_) 0)
    ((:tof_ground_filter_flag_3_ __tof_ground_filter_flag_3_) 0)
    ((:tof_ground_filter_flag_4_ __tof_ground_filter_flag_4_) 0)
    ((:tof_ground_filter_flag_5_ __tof_ground_filter_flag_5_) 0)
    ((:radar_up_output_mm __radar_up_output_mm) 0)
    ((:radar_down_output_mm __radar_down_output_mm) 0)
    ((:radar_hon_disp_range_cfg_mm __radar_hon_disp_range_cfg_mm) 0)
    ((:radar_down_disp_range_cfg_mm __radar_down_disp_range_cfg_mm) 0)
    ((:radar_up_disp_range_cfg_mm __radar_up_disp_range_cfg_mm) 0)
    ((:large_atti_no_oa __large_atti_no_oa) 0)
    ((:oa_status_top_bar __oa_status_top_bar) 0)
    ((:yaw __yaw) 0.0)
    ((:pitch __pitch) 0.0)
    ((:roll __roll) 0.0)
    ((:radar_graph_0_ __radar_graph_0_) 0)
    ((:radar_graph_1_ __radar_graph_1_) 0)
    ((:radar_graph_2_ __radar_graph_2_) 0)
    ((:radar_graph_3_ __radar_graph_3_) 0)
    ((:tof_radar_graph_data_mm_0_ __tof_radar_graph_data_mm_0_) 0)
    ((:tof_radar_graph_data_mm_1_ __tof_radar_graph_data_mm_1_) 0)
    ((:tof_radar_graph_data_mm_2_ __tof_radar_graph_data_mm_2_) 0)
    ((:tof_radar_graph_data_mm_3_ __tof_radar_graph_data_mm_3_) 0)
    ((:img_sts_ok_0_ __img_sts_ok_0_) 0)
    ((:img_sts_ok_1_ __img_sts_ok_1_) 0)
    ((:img_sts_ok_2_ __img_sts_ok_2_) 0)
    ((:img_sts_ok_3_ __img_sts_ok_3_) 0)
    ((:img_sts_ok_4_ __img_sts_ok_4_) 0)
    ((:img_sts_ok_5_ __img_sts_ok_5_) 0)
    ((:abnormal_dark_0_ __abnormal_dark_0_) 0)
    ((:abnormal_dark_1_ __abnormal_dark_1_) 0)
    ((:abnormal_dark_2_ __abnormal_dark_2_) 0)
    ((:abnormal_dark_3_ __abnormal_dark_3_) 0)
    ((:abnormal_dark_4_ __abnormal_dark_4_) 0)
    ((:abnormal_dark_5_ __abnormal_dark_5_) 0)
    ((:abnormal_bright_0_ __abnormal_bright_0_) 0)
    ((:abnormal_bright_1_ __abnormal_bright_1_) 0)
    ((:abnormal_bright_2_ __abnormal_bright_2_) 0)
    ((:abnormal_bright_3_ __abnormal_bright_3_) 0)
    ((:abnormal_bright_4_ __abnormal_bright_4_) 0)
    ((:abnormal_bright_5_ __abnormal_bright_5_) 0)
    ((:abnormal_similarity_0_ __abnormal_similarity_0_) 0)
    ((:abnormal_similarity_1_ __abnormal_similarity_1_) 0)
    ((:abnormal_similarity_2_ __abnormal_similarity_2_) 0)
    ((:abnormal_similarity_3_ __abnormal_similarity_3_) 0)
    ((:abnormal_similarity_4_ __abnormal_similarity_4_) 0)
    ((:abnormal_similarity_5_ __abnormal_similarity_5_) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _stereo_image_status_0_ (round __stereo_image_status_0_))
   (setq _stereo_image_status_1_ (round __stereo_image_status_1_))
   (setq _stereo_image_status_2_ (round __stereo_image_status_2_))
   (setq _stereo_image_status_3_ (round __stereo_image_status_3_))
   (setq _stereo_image_status_4_ (round __stereo_image_status_4_))
   (setq _stereo_image_status_5_ (round __stereo_image_status_5_))
   (setq _motor_active (round __motor_active))
   (setq _in_air (round __in_air))
   (setq _fc_ctrl_mode (round __fc_ctrl_mode))
   (setq _wm232_front_flag (round __wm232_front_flag))
   (setq _tof_ground_filter_flag_0_ (round __tof_ground_filter_flag_0_))
   (setq _tof_ground_filter_flag_1_ (round __tof_ground_filter_flag_1_))
   (setq _tof_ground_filter_flag_2_ (round __tof_ground_filter_flag_2_))
   (setq _tof_ground_filter_flag_3_ (round __tof_ground_filter_flag_3_))
   (setq _tof_ground_filter_flag_4_ (round __tof_ground_filter_flag_4_))
   (setq _tof_ground_filter_flag_5_ (round __tof_ground_filter_flag_5_))
   (setq _radar_up_output_mm (round __radar_up_output_mm))
   (setq _radar_down_output_mm (round __radar_down_output_mm))
   (setq _radar_hon_disp_range_cfg_mm (round __radar_hon_disp_range_cfg_mm))
   (setq _radar_down_disp_range_cfg_mm (round __radar_down_disp_range_cfg_mm))
   (setq _radar_up_disp_range_cfg_mm (round __radar_up_disp_range_cfg_mm))
   (setq _large_atti_no_oa (round __large_atti_no_oa))
   (setq _oa_status_top_bar (round __oa_status_top_bar))
   (setq _yaw (float __yaw))
   (setq _pitch (float __pitch))
   (setq _roll (float __roll))
   (setq _radar_graph_0_ (round __radar_graph_0_))
   (setq _radar_graph_1_ (round __radar_graph_1_))
   (setq _radar_graph_2_ (round __radar_graph_2_))
   (setq _radar_graph_3_ (round __radar_graph_3_))
   (setq _tof_radar_graph_data_mm_0_ (round __tof_radar_graph_data_mm_0_))
   (setq _tof_radar_graph_data_mm_1_ (round __tof_radar_graph_data_mm_1_))
   (setq _tof_radar_graph_data_mm_2_ (round __tof_radar_graph_data_mm_2_))
   (setq _tof_radar_graph_data_mm_3_ (round __tof_radar_graph_data_mm_3_))
   (setq _img_sts_ok_0_ (round __img_sts_ok_0_))
   (setq _img_sts_ok_1_ (round __img_sts_ok_1_))
   (setq _img_sts_ok_2_ (round __img_sts_ok_2_))
   (setq _img_sts_ok_3_ (round __img_sts_ok_3_))
   (setq _img_sts_ok_4_ (round __img_sts_ok_4_))
   (setq _img_sts_ok_5_ (round __img_sts_ok_5_))
   (setq _abnormal_dark_0_ (round __abnormal_dark_0_))
   (setq _abnormal_dark_1_ (round __abnormal_dark_1_))
   (setq _abnormal_dark_2_ (round __abnormal_dark_2_))
   (setq _abnormal_dark_3_ (round __abnormal_dark_3_))
   (setq _abnormal_dark_4_ (round __abnormal_dark_4_))
   (setq _abnormal_dark_5_ (round __abnormal_dark_5_))
   (setq _abnormal_bright_0_ (round __abnormal_bright_0_))
   (setq _abnormal_bright_1_ (round __abnormal_bright_1_))
   (setq _abnormal_bright_2_ (round __abnormal_bright_2_))
   (setq _abnormal_bright_3_ (round __abnormal_bright_3_))
   (setq _abnormal_bright_4_ (round __abnormal_bright_4_))
   (setq _abnormal_bright_5_ (round __abnormal_bright_5_))
   (setq _abnormal_similarity_0_ (round __abnormal_similarity_0_))
   (setq _abnormal_similarity_1_ (round __abnormal_similarity_1_))
   (setq _abnormal_similarity_2_ (round __abnormal_similarity_2_))
   (setq _abnormal_similarity_3_ (round __abnormal_similarity_3_))
   (setq _abnormal_similarity_4_ (round __abnormal_similarity_4_))
   (setq _abnormal_similarity_5_ (round __abnormal_similarity_5_))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:stereo_image_status_0_
   (&optional __stereo_image_status_0_)
   (if __stereo_image_status_0_ (setq _stereo_image_status_0_ __stereo_image_status_0_)) _stereo_image_status_0_)
  (:stereo_image_status_1_
   (&optional __stereo_image_status_1_)
   (if __stereo_image_status_1_ (setq _stereo_image_status_1_ __stereo_image_status_1_)) _stereo_image_status_1_)
  (:stereo_image_status_2_
   (&optional __stereo_image_status_2_)
   (if __stereo_image_status_2_ (setq _stereo_image_status_2_ __stereo_image_status_2_)) _stereo_image_status_2_)
  (:stereo_image_status_3_
   (&optional __stereo_image_status_3_)
   (if __stereo_image_status_3_ (setq _stereo_image_status_3_ __stereo_image_status_3_)) _stereo_image_status_3_)
  (:stereo_image_status_4_
   (&optional __stereo_image_status_4_)
   (if __stereo_image_status_4_ (setq _stereo_image_status_4_ __stereo_image_status_4_)) _stereo_image_status_4_)
  (:stereo_image_status_5_
   (&optional __stereo_image_status_5_)
   (if __stereo_image_status_5_ (setq _stereo_image_status_5_ __stereo_image_status_5_)) _stereo_image_status_5_)
  (:motor_active
   (&optional __motor_active)
   (if __motor_active (setq _motor_active __motor_active)) _motor_active)
  (:in_air
   (&optional __in_air)
   (if __in_air (setq _in_air __in_air)) _in_air)
  (:fc_ctrl_mode
   (&optional __fc_ctrl_mode)
   (if __fc_ctrl_mode (setq _fc_ctrl_mode __fc_ctrl_mode)) _fc_ctrl_mode)
  (:wm232_front_flag
   (&optional __wm232_front_flag)
   (if __wm232_front_flag (setq _wm232_front_flag __wm232_front_flag)) _wm232_front_flag)
  (:tof_ground_filter_flag_0_
   (&optional __tof_ground_filter_flag_0_)
   (if __tof_ground_filter_flag_0_ (setq _tof_ground_filter_flag_0_ __tof_ground_filter_flag_0_)) _tof_ground_filter_flag_0_)
  (:tof_ground_filter_flag_1_
   (&optional __tof_ground_filter_flag_1_)
   (if __tof_ground_filter_flag_1_ (setq _tof_ground_filter_flag_1_ __tof_ground_filter_flag_1_)) _tof_ground_filter_flag_1_)
  (:tof_ground_filter_flag_2_
   (&optional __tof_ground_filter_flag_2_)
   (if __tof_ground_filter_flag_2_ (setq _tof_ground_filter_flag_2_ __tof_ground_filter_flag_2_)) _tof_ground_filter_flag_2_)
  (:tof_ground_filter_flag_3_
   (&optional __tof_ground_filter_flag_3_)
   (if __tof_ground_filter_flag_3_ (setq _tof_ground_filter_flag_3_ __tof_ground_filter_flag_3_)) _tof_ground_filter_flag_3_)
  (:tof_ground_filter_flag_4_
   (&optional __tof_ground_filter_flag_4_)
   (if __tof_ground_filter_flag_4_ (setq _tof_ground_filter_flag_4_ __tof_ground_filter_flag_4_)) _tof_ground_filter_flag_4_)
  (:tof_ground_filter_flag_5_
   (&optional __tof_ground_filter_flag_5_)
   (if __tof_ground_filter_flag_5_ (setq _tof_ground_filter_flag_5_ __tof_ground_filter_flag_5_)) _tof_ground_filter_flag_5_)
  (:radar_up_output_mm
   (&optional __radar_up_output_mm)
   (if __radar_up_output_mm (setq _radar_up_output_mm __radar_up_output_mm)) _radar_up_output_mm)
  (:radar_down_output_mm
   (&optional __radar_down_output_mm)
   (if __radar_down_output_mm (setq _radar_down_output_mm __radar_down_output_mm)) _radar_down_output_mm)
  (:radar_hon_disp_range_cfg_mm
   (&optional __radar_hon_disp_range_cfg_mm)
   (if __radar_hon_disp_range_cfg_mm (setq _radar_hon_disp_range_cfg_mm __radar_hon_disp_range_cfg_mm)) _radar_hon_disp_range_cfg_mm)
  (:radar_down_disp_range_cfg_mm
   (&optional __radar_down_disp_range_cfg_mm)
   (if __radar_down_disp_range_cfg_mm (setq _radar_down_disp_range_cfg_mm __radar_down_disp_range_cfg_mm)) _radar_down_disp_range_cfg_mm)
  (:radar_up_disp_range_cfg_mm
   (&optional __radar_up_disp_range_cfg_mm)
   (if __radar_up_disp_range_cfg_mm (setq _radar_up_disp_range_cfg_mm __radar_up_disp_range_cfg_mm)) _radar_up_disp_range_cfg_mm)
  (:large_atti_no_oa
   (&optional __large_atti_no_oa)
   (if __large_atti_no_oa (setq _large_atti_no_oa __large_atti_no_oa)) _large_atti_no_oa)
  (:oa_status_top_bar
   (&optional __oa_status_top_bar)
   (if __oa_status_top_bar (setq _oa_status_top_bar __oa_status_top_bar)) _oa_status_top_bar)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:radar_graph_0_
   (&optional __radar_graph_0_)
   (if __radar_graph_0_ (setq _radar_graph_0_ __radar_graph_0_)) _radar_graph_0_)
  (:radar_graph_1_
   (&optional __radar_graph_1_)
   (if __radar_graph_1_ (setq _radar_graph_1_ __radar_graph_1_)) _radar_graph_1_)
  (:radar_graph_2_
   (&optional __radar_graph_2_)
   (if __radar_graph_2_ (setq _radar_graph_2_ __radar_graph_2_)) _radar_graph_2_)
  (:radar_graph_3_
   (&optional __radar_graph_3_)
   (if __radar_graph_3_ (setq _radar_graph_3_ __radar_graph_3_)) _radar_graph_3_)
  (:tof_radar_graph_data_mm_0_
   (&optional __tof_radar_graph_data_mm_0_)
   (if __tof_radar_graph_data_mm_0_ (setq _tof_radar_graph_data_mm_0_ __tof_radar_graph_data_mm_0_)) _tof_radar_graph_data_mm_0_)
  (:tof_radar_graph_data_mm_1_
   (&optional __tof_radar_graph_data_mm_1_)
   (if __tof_radar_graph_data_mm_1_ (setq _tof_radar_graph_data_mm_1_ __tof_radar_graph_data_mm_1_)) _tof_radar_graph_data_mm_1_)
  (:tof_radar_graph_data_mm_2_
   (&optional __tof_radar_graph_data_mm_2_)
   (if __tof_radar_graph_data_mm_2_ (setq _tof_radar_graph_data_mm_2_ __tof_radar_graph_data_mm_2_)) _tof_radar_graph_data_mm_2_)
  (:tof_radar_graph_data_mm_3_
   (&optional __tof_radar_graph_data_mm_3_)
   (if __tof_radar_graph_data_mm_3_ (setq _tof_radar_graph_data_mm_3_ __tof_radar_graph_data_mm_3_)) _tof_radar_graph_data_mm_3_)
  (:img_sts_ok_0_
   (&optional __img_sts_ok_0_)
   (if __img_sts_ok_0_ (setq _img_sts_ok_0_ __img_sts_ok_0_)) _img_sts_ok_0_)
  (:img_sts_ok_1_
   (&optional __img_sts_ok_1_)
   (if __img_sts_ok_1_ (setq _img_sts_ok_1_ __img_sts_ok_1_)) _img_sts_ok_1_)
  (:img_sts_ok_2_
   (&optional __img_sts_ok_2_)
   (if __img_sts_ok_2_ (setq _img_sts_ok_2_ __img_sts_ok_2_)) _img_sts_ok_2_)
  (:img_sts_ok_3_
   (&optional __img_sts_ok_3_)
   (if __img_sts_ok_3_ (setq _img_sts_ok_3_ __img_sts_ok_3_)) _img_sts_ok_3_)
  (:img_sts_ok_4_
   (&optional __img_sts_ok_4_)
   (if __img_sts_ok_4_ (setq _img_sts_ok_4_ __img_sts_ok_4_)) _img_sts_ok_4_)
  (:img_sts_ok_5_
   (&optional __img_sts_ok_5_)
   (if __img_sts_ok_5_ (setq _img_sts_ok_5_ __img_sts_ok_5_)) _img_sts_ok_5_)
  (:abnormal_dark_0_
   (&optional __abnormal_dark_0_)
   (if __abnormal_dark_0_ (setq _abnormal_dark_0_ __abnormal_dark_0_)) _abnormal_dark_0_)
  (:abnormal_dark_1_
   (&optional __abnormal_dark_1_)
   (if __abnormal_dark_1_ (setq _abnormal_dark_1_ __abnormal_dark_1_)) _abnormal_dark_1_)
  (:abnormal_dark_2_
   (&optional __abnormal_dark_2_)
   (if __abnormal_dark_2_ (setq _abnormal_dark_2_ __abnormal_dark_2_)) _abnormal_dark_2_)
  (:abnormal_dark_3_
   (&optional __abnormal_dark_3_)
   (if __abnormal_dark_3_ (setq _abnormal_dark_3_ __abnormal_dark_3_)) _abnormal_dark_3_)
  (:abnormal_dark_4_
   (&optional __abnormal_dark_4_)
   (if __abnormal_dark_4_ (setq _abnormal_dark_4_ __abnormal_dark_4_)) _abnormal_dark_4_)
  (:abnormal_dark_5_
   (&optional __abnormal_dark_5_)
   (if __abnormal_dark_5_ (setq _abnormal_dark_5_ __abnormal_dark_5_)) _abnormal_dark_5_)
  (:abnormal_bright_0_
   (&optional __abnormal_bright_0_)
   (if __abnormal_bright_0_ (setq _abnormal_bright_0_ __abnormal_bright_0_)) _abnormal_bright_0_)
  (:abnormal_bright_1_
   (&optional __abnormal_bright_1_)
   (if __abnormal_bright_1_ (setq _abnormal_bright_1_ __abnormal_bright_1_)) _abnormal_bright_1_)
  (:abnormal_bright_2_
   (&optional __abnormal_bright_2_)
   (if __abnormal_bright_2_ (setq _abnormal_bright_2_ __abnormal_bright_2_)) _abnormal_bright_2_)
  (:abnormal_bright_3_
   (&optional __abnormal_bright_3_)
   (if __abnormal_bright_3_ (setq _abnormal_bright_3_ __abnormal_bright_3_)) _abnormal_bright_3_)
  (:abnormal_bright_4_
   (&optional __abnormal_bright_4_)
   (if __abnormal_bright_4_ (setq _abnormal_bright_4_ __abnormal_bright_4_)) _abnormal_bright_4_)
  (:abnormal_bright_5_
   (&optional __abnormal_bright_5_)
   (if __abnormal_bright_5_ (setq _abnormal_bright_5_ __abnormal_bright_5_)) _abnormal_bright_5_)
  (:abnormal_similarity_0_
   (&optional __abnormal_similarity_0_)
   (if __abnormal_similarity_0_ (setq _abnormal_similarity_0_ __abnormal_similarity_0_)) _abnormal_similarity_0_)
  (:abnormal_similarity_1_
   (&optional __abnormal_similarity_1_)
   (if __abnormal_similarity_1_ (setq _abnormal_similarity_1_ __abnormal_similarity_1_)) _abnormal_similarity_1_)
  (:abnormal_similarity_2_
   (&optional __abnormal_similarity_2_)
   (if __abnormal_similarity_2_ (setq _abnormal_similarity_2_ __abnormal_similarity_2_)) _abnormal_similarity_2_)
  (:abnormal_similarity_3_
   (&optional __abnormal_similarity_3_)
   (if __abnormal_similarity_3_ (setq _abnormal_similarity_3_ __abnormal_similarity_3_)) _abnormal_similarity_3_)
  (:abnormal_similarity_4_
   (&optional __abnormal_similarity_4_)
   (if __abnormal_similarity_4_ (setq _abnormal_similarity_4_ __abnormal_similarity_4_)) _abnormal_similarity_4_)
  (:abnormal_similarity_5_
   (&optional __abnormal_similarity_5_)
   (if __abnormal_similarity_5_ (setq _abnormal_similarity_5_ __abnormal_similarity_5_)) _abnormal_similarity_5_)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _stereo_image_status_0_
    1
    ;; uint8 _stereo_image_status_1_
    1
    ;; uint8 _stereo_image_status_2_
    1
    ;; uint8 _stereo_image_status_3_
    1
    ;; uint8 _stereo_image_status_4_
    1
    ;; uint8 _stereo_image_status_5_
    1
    ;; uint8 _motor_active
    1
    ;; uint8 _in_air
    1
    ;; uint8 _fc_ctrl_mode
    1
    ;; uint8 _wm232_front_flag
    1
    ;; uint8 _tof_ground_filter_flag_0_
    1
    ;; uint8 _tof_ground_filter_flag_1_
    1
    ;; uint8 _tof_ground_filter_flag_2_
    1
    ;; uint8 _tof_ground_filter_flag_3_
    1
    ;; uint8 _tof_ground_filter_flag_4_
    1
    ;; uint8 _tof_ground_filter_flag_5_
    1
    ;; uint16 _radar_up_output_mm
    2
    ;; uint16 _radar_down_output_mm
    2
    ;; uint16 _radar_hon_disp_range_cfg_mm
    2
    ;; uint16 _radar_down_disp_range_cfg_mm
    2
    ;; uint16 _radar_up_disp_range_cfg_mm
    2
    ;; uint8 _large_atti_no_oa
    1
    ;; uint16 _oa_status_top_bar
    2
    ;; float32 _yaw
    4
    ;; float32 _pitch
    4
    ;; float32 _roll
    4
    ;; int32 _radar_graph_0_
    4
    ;; int32 _radar_graph_1_
    4
    ;; int32 _radar_graph_2_
    4
    ;; int32 _radar_graph_3_
    4
    ;; uint16 _tof_radar_graph_data_mm_0_
    2
    ;; uint16 _tof_radar_graph_data_mm_1_
    2
    ;; uint16 _tof_radar_graph_data_mm_2_
    2
    ;; uint16 _tof_radar_graph_data_mm_3_
    2
    ;; uint8 _img_sts_ok_0_
    1
    ;; uint8 _img_sts_ok_1_
    1
    ;; uint8 _img_sts_ok_2_
    1
    ;; uint8 _img_sts_ok_3_
    1
    ;; uint8 _img_sts_ok_4_
    1
    ;; uint8 _img_sts_ok_5_
    1
    ;; uint8 _abnormal_dark_0_
    1
    ;; uint8 _abnormal_dark_1_
    1
    ;; uint8 _abnormal_dark_2_
    1
    ;; uint8 _abnormal_dark_3_
    1
    ;; uint8 _abnormal_dark_4_
    1
    ;; uint8 _abnormal_dark_5_
    1
    ;; uint8 _abnormal_bright_0_
    1
    ;; uint8 _abnormal_bright_1_
    1
    ;; uint8 _abnormal_bright_2_
    1
    ;; uint8 _abnormal_bright_3_
    1
    ;; uint8 _abnormal_bright_4_
    1
    ;; uint8 _abnormal_bright_5_
    1
    ;; uint8 _abnormal_similarity_0_
    1
    ;; uint8 _abnormal_similarity_1_
    1
    ;; uint8 _abnormal_similarity_2_
    1
    ;; uint8 _abnormal_similarity_3_
    1
    ;; uint8 _abnormal_similarity_4_
    1
    ;; uint8 _abnormal_similarity_5_
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _stereo_image_status_0_
       (write-byte _stereo_image_status_0_ s)
     ;; uint8 _stereo_image_status_1_
       (write-byte _stereo_image_status_1_ s)
     ;; uint8 _stereo_image_status_2_
       (write-byte _stereo_image_status_2_ s)
     ;; uint8 _stereo_image_status_3_
       (write-byte _stereo_image_status_3_ s)
     ;; uint8 _stereo_image_status_4_
       (write-byte _stereo_image_status_4_ s)
     ;; uint8 _stereo_image_status_5_
       (write-byte _stereo_image_status_5_ s)
     ;; uint8 _motor_active
       (write-byte _motor_active s)
     ;; uint8 _in_air
       (write-byte _in_air s)
     ;; uint8 _fc_ctrl_mode
       (write-byte _fc_ctrl_mode s)
     ;; uint8 _wm232_front_flag
       (write-byte _wm232_front_flag s)
     ;; uint8 _tof_ground_filter_flag_0_
       (write-byte _tof_ground_filter_flag_0_ s)
     ;; uint8 _tof_ground_filter_flag_1_
       (write-byte _tof_ground_filter_flag_1_ s)
     ;; uint8 _tof_ground_filter_flag_2_
       (write-byte _tof_ground_filter_flag_2_ s)
     ;; uint8 _tof_ground_filter_flag_3_
       (write-byte _tof_ground_filter_flag_3_ s)
     ;; uint8 _tof_ground_filter_flag_4_
       (write-byte _tof_ground_filter_flag_4_ s)
     ;; uint8 _tof_ground_filter_flag_5_
       (write-byte _tof_ground_filter_flag_5_ s)
     ;; uint16 _radar_up_output_mm
       (write-word _radar_up_output_mm s)
     ;; uint16 _radar_down_output_mm
       (write-word _radar_down_output_mm s)
     ;; uint16 _radar_hon_disp_range_cfg_mm
       (write-word _radar_hon_disp_range_cfg_mm s)
     ;; uint16 _radar_down_disp_range_cfg_mm
       (write-word _radar_down_disp_range_cfg_mm s)
     ;; uint16 _radar_up_disp_range_cfg_mm
       (write-word _radar_up_disp_range_cfg_mm s)
     ;; uint8 _large_atti_no_oa
       (write-byte _large_atti_no_oa s)
     ;; uint16 _oa_status_top_bar
       (write-word _oa_status_top_bar s)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _radar_graph_0_
       (write-long _radar_graph_0_ s)
     ;; int32 _radar_graph_1_
       (write-long _radar_graph_1_ s)
     ;; int32 _radar_graph_2_
       (write-long _radar_graph_2_ s)
     ;; int32 _radar_graph_3_
       (write-long _radar_graph_3_ s)
     ;; uint16 _tof_radar_graph_data_mm_0_
       (write-word _tof_radar_graph_data_mm_0_ s)
     ;; uint16 _tof_radar_graph_data_mm_1_
       (write-word _tof_radar_graph_data_mm_1_ s)
     ;; uint16 _tof_radar_graph_data_mm_2_
       (write-word _tof_radar_graph_data_mm_2_ s)
     ;; uint16 _tof_radar_graph_data_mm_3_
       (write-word _tof_radar_graph_data_mm_3_ s)
     ;; uint8 _img_sts_ok_0_
       (write-byte _img_sts_ok_0_ s)
     ;; uint8 _img_sts_ok_1_
       (write-byte _img_sts_ok_1_ s)
     ;; uint8 _img_sts_ok_2_
       (write-byte _img_sts_ok_2_ s)
     ;; uint8 _img_sts_ok_3_
       (write-byte _img_sts_ok_3_ s)
     ;; uint8 _img_sts_ok_4_
       (write-byte _img_sts_ok_4_ s)
     ;; uint8 _img_sts_ok_5_
       (write-byte _img_sts_ok_5_ s)
     ;; uint8 _abnormal_dark_0_
       (write-byte _abnormal_dark_0_ s)
     ;; uint8 _abnormal_dark_1_
       (write-byte _abnormal_dark_1_ s)
     ;; uint8 _abnormal_dark_2_
       (write-byte _abnormal_dark_2_ s)
     ;; uint8 _abnormal_dark_3_
       (write-byte _abnormal_dark_3_ s)
     ;; uint8 _abnormal_dark_4_
       (write-byte _abnormal_dark_4_ s)
     ;; uint8 _abnormal_dark_5_
       (write-byte _abnormal_dark_5_ s)
     ;; uint8 _abnormal_bright_0_
       (write-byte _abnormal_bright_0_ s)
     ;; uint8 _abnormal_bright_1_
       (write-byte _abnormal_bright_1_ s)
     ;; uint8 _abnormal_bright_2_
       (write-byte _abnormal_bright_2_ s)
     ;; uint8 _abnormal_bright_3_
       (write-byte _abnormal_bright_3_ s)
     ;; uint8 _abnormal_bright_4_
       (write-byte _abnormal_bright_4_ s)
     ;; uint8 _abnormal_bright_5_
       (write-byte _abnormal_bright_5_ s)
     ;; uint8 _abnormal_similarity_0_
       (write-byte _abnormal_similarity_0_ s)
     ;; uint8 _abnormal_similarity_1_
       (write-byte _abnormal_similarity_1_ s)
     ;; uint8 _abnormal_similarity_2_
       (write-byte _abnormal_similarity_2_ s)
     ;; uint8 _abnormal_similarity_3_
       (write-byte _abnormal_similarity_3_ s)
     ;; uint8 _abnormal_similarity_4_
       (write-byte _abnormal_similarity_4_ s)
     ;; uint8 _abnormal_similarity_5_
       (write-byte _abnormal_similarity_5_ s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _stereo_image_status_0_
     (setq _stereo_image_status_0_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _stereo_image_status_1_
     (setq _stereo_image_status_1_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _stereo_image_status_2_
     (setq _stereo_image_status_2_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _stereo_image_status_3_
     (setq _stereo_image_status_3_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _stereo_image_status_4_
     (setq _stereo_image_status_4_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _stereo_image_status_5_
     (setq _stereo_image_status_5_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _motor_active
     (setq _motor_active (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _in_air
     (setq _in_air (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _fc_ctrl_mode
     (setq _fc_ctrl_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _wm232_front_flag
     (setq _wm232_front_flag (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _tof_ground_filter_flag_0_
     (setq _tof_ground_filter_flag_0_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _tof_ground_filter_flag_1_
     (setq _tof_ground_filter_flag_1_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _tof_ground_filter_flag_2_
     (setq _tof_ground_filter_flag_2_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _tof_ground_filter_flag_3_
     (setq _tof_ground_filter_flag_3_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _tof_ground_filter_flag_4_
     (setq _tof_ground_filter_flag_4_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _tof_ground_filter_flag_5_
     (setq _tof_ground_filter_flag_5_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _radar_up_output_mm
     (setq _radar_up_output_mm (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _radar_down_output_mm
     (setq _radar_down_output_mm (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _radar_hon_disp_range_cfg_mm
     (setq _radar_hon_disp_range_cfg_mm (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _radar_down_disp_range_cfg_mm
     (setq _radar_down_disp_range_cfg_mm (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _radar_up_disp_range_cfg_mm
     (setq _radar_up_disp_range_cfg_mm (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _large_atti_no_oa
     (setq _large_atti_no_oa (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _oa_status_top_bar
     (setq _oa_status_top_bar (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _radar_graph_0_
     (setq _radar_graph_0_ (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _radar_graph_1_
     (setq _radar_graph_1_ (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _radar_graph_2_
     (setq _radar_graph_2_ (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _radar_graph_3_
     (setq _radar_graph_3_ (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _tof_radar_graph_data_mm_0_
     (setq _tof_radar_graph_data_mm_0_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _tof_radar_graph_data_mm_1_
     (setq _tof_radar_graph_data_mm_1_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _tof_radar_graph_data_mm_2_
     (setq _tof_radar_graph_data_mm_2_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _tof_radar_graph_data_mm_3_
     (setq _tof_radar_graph_data_mm_3_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _img_sts_ok_0_
     (setq _img_sts_ok_0_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _img_sts_ok_1_
     (setq _img_sts_ok_1_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _img_sts_ok_2_
     (setq _img_sts_ok_2_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _img_sts_ok_3_
     (setq _img_sts_ok_3_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _img_sts_ok_4_
     (setq _img_sts_ok_4_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _img_sts_ok_5_
     (setq _img_sts_ok_5_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_dark_0_
     (setq _abnormal_dark_0_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_dark_1_
     (setq _abnormal_dark_1_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_dark_2_
     (setq _abnormal_dark_2_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_dark_3_
     (setq _abnormal_dark_3_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_dark_4_
     (setq _abnormal_dark_4_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_dark_5_
     (setq _abnormal_dark_5_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_bright_0_
     (setq _abnormal_bright_0_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_bright_1_
     (setq _abnormal_bright_1_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_bright_2_
     (setq _abnormal_bright_2_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_bright_3_
     (setq _abnormal_bright_3_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_bright_4_
     (setq _abnormal_bright_4_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_bright_5_
     (setq _abnormal_bright_5_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_similarity_0_
     (setq _abnormal_similarity_0_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_similarity_1_
     (setq _abnormal_similarity_1_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_similarity_2_
     (setq _abnormal_similarity_2_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_similarity_3_
     (setq _abnormal_similarity_3_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_similarity_4_
     (setq _abnormal_similarity_4_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abnormal_similarity_5_
     (setq _abnormal_similarity_5_ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get quadrotor_msgs::oa_manager_debug :md5sum-) "2fe2189ab02dae1ff093d243f6f55759")
(setf (get quadrotor_msgs::oa_manager_debug :datatype-) "quadrotor_msgs/oa_manager_debug")
(setf (get quadrotor_msgs::oa_manager_debug :definition-)
      "std_msgs/Header header
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

")



(provide :quadrotor_msgs/oa_manager_debug "2fe2189ab02dae1ff093d243f6f55759")


