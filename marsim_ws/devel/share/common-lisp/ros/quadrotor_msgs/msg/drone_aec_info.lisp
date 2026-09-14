; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude drone_aec_info.msg.html

(cl:defclass <drone_aec_info> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (expo_time_ms
    :reader expo_time_ms
    :initarg :expo_time_ms
    :type cl:float
    :initform 0.0)
   (expo_gain
    :reader expo_gain
    :initarg :expo_gain
    :type cl:float
    :initform 0.0)
   (scene_lum_lux
    :reader scene_lum_lux
    :initarg :scene_lum_lux
    :type cl:float
    :initform 0.0)
   (avg_lum
    :reader avg_lum
    :initarg :avg_lum
    :type cl:float
    :initform 0.0))
)

(cl:defclass drone_aec_info (<drone_aec_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <drone_aec_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'drone_aec_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<drone_aec_info> is deprecated: use quadrotor_msgs-msg:drone_aec_info instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <drone_aec_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'expo_time_ms-val :lambda-list '(m))
(cl:defmethod expo_time_ms-val ((m <drone_aec_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:expo_time_ms-val is deprecated.  Use quadrotor_msgs-msg:expo_time_ms instead.")
  (expo_time_ms m))

(cl:ensure-generic-function 'expo_gain-val :lambda-list '(m))
(cl:defmethod expo_gain-val ((m <drone_aec_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:expo_gain-val is deprecated.  Use quadrotor_msgs-msg:expo_gain instead.")
  (expo_gain m))

(cl:ensure-generic-function 'scene_lum_lux-val :lambda-list '(m))
(cl:defmethod scene_lum_lux-val ((m <drone_aec_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:scene_lum_lux-val is deprecated.  Use quadrotor_msgs-msg:scene_lum_lux instead.")
  (scene_lum_lux m))

(cl:ensure-generic-function 'avg_lum-val :lambda-list '(m))
(cl:defmethod avg_lum-val ((m <drone_aec_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:avg_lum-val is deprecated.  Use quadrotor_msgs-msg:avg_lum instead.")
  (avg_lum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <drone_aec_info>) ostream)
  "Serializes a message object of type '<drone_aec_info>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'expo_time_ms))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'expo_gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'scene_lum_lux))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'avg_lum))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <drone_aec_info>) istream)
  "Deserializes a message object of type '<drone_aec_info>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'expo_time_ms) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'expo_gain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'scene_lum_lux) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'avg_lum) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<drone_aec_info>)))
  "Returns string type for a message object of type '<drone_aec_info>"
  "quadrotor_msgs/drone_aec_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'drone_aec_info)))
  "Returns string type for a message object of type 'drone_aec_info"
  "quadrotor_msgs/drone_aec_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<drone_aec_info>)))
  "Returns md5sum for a message object of type '<drone_aec_info>"
  "c3d90219ba162118512fb07f93a5de00")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'drone_aec_info)))
  "Returns md5sum for a message object of type 'drone_aec_info"
  "c3d90219ba162118512fb07f93a5de00")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<drone_aec_info>)))
  "Returns full string definition for message of type '<drone_aec_info>"
  (cl:format cl:nil "std_msgs/Header header~%float32 expo_time_ms~%float32 expo_gain~%float32 scene_lum_lux~%float32 avg_lum~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'drone_aec_info)))
  "Returns full string definition for message of type 'drone_aec_info"
  (cl:format cl:nil "std_msgs/Header header~%float32 expo_time_ms~%float32 expo_gain~%float32 scene_lum_lux~%float32 avg_lum~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <drone_aec_info>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <drone_aec_info>))
  "Converts a ROS message object to a list"
  (cl:list 'drone_aec_info
    (cl:cons ':header (header msg))
    (cl:cons ':expo_time_ms (expo_time_ms msg))
    (cl:cons ':expo_gain (expo_gain msg))
    (cl:cons ':scene_lum_lux (scene_lum_lux msg))
    (cl:cons ':avg_lum (avg_lum msg))
))
