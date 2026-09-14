;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::esdf_map)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'esdf_map (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::ESDF_MAP")
  (make-package "QUADROTOR_MSGS::ESDF_MAP"))

(in-package "ROS")
;;//! \htmlinclude esdf_map.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::esdf_map
  :super ros::object
  :slots (_header _map_plantform _map_version _map_log_version _map_log_type ))

(defmethod quadrotor_msgs::esdf_map
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:map_plantform __map_plantform) 0)
    ((:map_version __map_version) 0)
    ((:map_log_version __map_log_version) 0)
    ((:map_log_type __map_log_type) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _map_plantform (round __map_plantform))
   (setq _map_version (round __map_version))
   (setq _map_log_version (round __map_log_version))
   (setq _map_log_type (round __map_log_type))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:map_plantform
   (&optional __map_plantform)
   (if __map_plantform (setq _map_plantform __map_plantform)) _map_plantform)
  (:map_version
   (&optional __map_version)
   (if __map_version (setq _map_version __map_version)) _map_version)
  (:map_log_version
   (&optional __map_log_version)
   (if __map_log_version (setq _map_log_version __map_log_version)) _map_log_version)
  (:map_log_type
   (&optional __map_log_type)
   (if __map_log_type (setq _map_log_type __map_log_type)) _map_log_type)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _map_plantform
    2
    ;; uint8 _map_version
    1
    ;; uint8 _map_log_version
    1
    ;; uint8 _map_log_type
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _map_plantform
       (write-word _map_plantform s)
     ;; uint8 _map_version
       (write-byte _map_version s)
     ;; uint8 _map_log_version
       (write-byte _map_log_version s)
     ;; uint8 _map_log_type
       (write-byte _map_log_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _map_plantform
     (setq _map_plantform (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _map_version
     (setq _map_version (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _map_log_version
     (setq _map_log_version (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _map_log_type
     (setq _map_log_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get quadrotor_msgs::esdf_map :md5sum-) "b7e611753bf33b0e49ba1e83fb044535")
(setf (get quadrotor_msgs::esdf_map :datatype-) "quadrotor_msgs/esdf_map")
(setf (get quadrotor_msgs::esdf_map :definition-)
      "Header header
uint16 map_plantform
uint8 map_version
uint8 map_log_version
uint8 map_log_type


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



(provide :quadrotor_msgs/esdf_map "b7e611753bf33b0e49ba1e83fb044535")


