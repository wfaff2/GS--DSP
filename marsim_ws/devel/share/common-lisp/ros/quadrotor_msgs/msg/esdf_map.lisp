; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude esdf_map.msg.html

(cl:defclass <esdf_map> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (map_plantform
    :reader map_plantform
    :initarg :map_plantform
    :type cl:fixnum
    :initform 0)
   (map_version
    :reader map_version
    :initarg :map_version
    :type cl:fixnum
    :initform 0)
   (map_log_version
    :reader map_log_version
    :initarg :map_log_version
    :type cl:fixnum
    :initform 0)
   (map_log_type
    :reader map_log_type
    :initarg :map_log_type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass esdf_map (<esdf_map>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <esdf_map>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'esdf_map)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<esdf_map> is deprecated: use quadrotor_msgs-msg:esdf_map instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <esdf_map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'map_plantform-val :lambda-list '(m))
(cl:defmethod map_plantform-val ((m <esdf_map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:map_plantform-val is deprecated.  Use quadrotor_msgs-msg:map_plantform instead.")
  (map_plantform m))

(cl:ensure-generic-function 'map_version-val :lambda-list '(m))
(cl:defmethod map_version-val ((m <esdf_map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:map_version-val is deprecated.  Use quadrotor_msgs-msg:map_version instead.")
  (map_version m))

(cl:ensure-generic-function 'map_log_version-val :lambda-list '(m))
(cl:defmethod map_log_version-val ((m <esdf_map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:map_log_version-val is deprecated.  Use quadrotor_msgs-msg:map_log_version instead.")
  (map_log_version m))

(cl:ensure-generic-function 'map_log_type-val :lambda-list '(m))
(cl:defmethod map_log_type-val ((m <esdf_map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:map_log_type-val is deprecated.  Use quadrotor_msgs-msg:map_log_type instead.")
  (map_log_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <esdf_map>) ostream)
  "Serializes a message object of type '<esdf_map>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'map_plantform)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'map_plantform)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'map_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'map_log_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'map_log_type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <esdf_map>) istream)
  "Deserializes a message object of type '<esdf_map>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'map_plantform)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'map_plantform)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'map_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'map_log_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'map_log_type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<esdf_map>)))
  "Returns string type for a message object of type '<esdf_map>"
  "quadrotor_msgs/esdf_map")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'esdf_map)))
  "Returns string type for a message object of type 'esdf_map"
  "quadrotor_msgs/esdf_map")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<esdf_map>)))
  "Returns md5sum for a message object of type '<esdf_map>"
  "b7e611753bf33b0e49ba1e83fb044535")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'esdf_map)))
  "Returns md5sum for a message object of type 'esdf_map"
  "b7e611753bf33b0e49ba1e83fb044535")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<esdf_map>)))
  "Returns full string definition for message of type '<esdf_map>"
  (cl:format cl:nil "Header header~%uint16 map_plantform~%uint8 map_version~%uint8 map_log_version~%uint8 map_log_type~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'esdf_map)))
  "Returns full string definition for message of type 'esdf_map"
  (cl:format cl:nil "Header header~%uint16 map_plantform~%uint8 map_version~%uint8 map_log_version~%uint8 map_log_type~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <esdf_map>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <esdf_map>))
  "Converts a ROS message object to a list"
  (cl:list 'esdf_map
    (cl:cons ':header (header msg))
    (cl:cons ':map_plantform (map_plantform msg))
    (cl:cons ':map_version (map_version msg))
    (cl:cons ':map_log_version (map_log_version msg))
    (cl:cons ':map_log_type (map_log_type msg))
))
