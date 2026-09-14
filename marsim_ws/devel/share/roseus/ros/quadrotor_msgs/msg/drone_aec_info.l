;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::drone_aec_info)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'drone_aec_info (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::DRONE_AEC_INFO")
  (make-package "QUADROTOR_MSGS::DRONE_AEC_INFO"))

(in-package "ROS")
;;//! \htmlinclude drone_aec_info.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::drone_aec_info
  :super ros::object
  :slots (_header _expo_time_ms _expo_gain _scene_lum_lux _avg_lum ))

(defmethod quadrotor_msgs::drone_aec_info
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:expo_time_ms __expo_time_ms) 0.0)
    ((:expo_gain __expo_gain) 0.0)
    ((:scene_lum_lux __scene_lum_lux) 0.0)
    ((:avg_lum __avg_lum) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _expo_time_ms (float __expo_time_ms))
   (setq _expo_gain (float __expo_gain))
   (setq _scene_lum_lux (float __scene_lum_lux))
   (setq _avg_lum (float __avg_lum))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:expo_time_ms
   (&optional __expo_time_ms)
   (if __expo_time_ms (setq _expo_time_ms __expo_time_ms)) _expo_time_ms)
  (:expo_gain
   (&optional __expo_gain)
   (if __expo_gain (setq _expo_gain __expo_gain)) _expo_gain)
  (:scene_lum_lux
   (&optional __scene_lum_lux)
   (if __scene_lum_lux (setq _scene_lum_lux __scene_lum_lux)) _scene_lum_lux)
  (:avg_lum
   (&optional __avg_lum)
   (if __avg_lum (setq _avg_lum __avg_lum)) _avg_lum)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _expo_time_ms
    4
    ;; float32 _expo_gain
    4
    ;; float32 _scene_lum_lux
    4
    ;; float32 _avg_lum
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _expo_time_ms
       (sys::poke _expo_time_ms (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _expo_gain
       (sys::poke _expo_gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _scene_lum_lux
       (sys::poke _scene_lum_lux (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _avg_lum
       (sys::poke _avg_lum (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _expo_time_ms
     (setq _expo_time_ms (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _expo_gain
     (setq _expo_gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _scene_lum_lux
     (setq _scene_lum_lux (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _avg_lum
     (setq _avg_lum (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get quadrotor_msgs::drone_aec_info :md5sum-) "c3d90219ba162118512fb07f93a5de00")
(setf (get quadrotor_msgs::drone_aec_info :datatype-) "quadrotor_msgs/drone_aec_info")
(setf (get quadrotor_msgs::drone_aec_info :definition-)
      "std_msgs/Header header
float32 expo_time_ms
float32 expo_gain
float32 scene_lum_lux
float32 avg_lum

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



(provide :quadrotor_msgs/drone_aec_info "c3d90219ba162118512fb07f93a5de00")


