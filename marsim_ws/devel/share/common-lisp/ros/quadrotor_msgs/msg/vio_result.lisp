; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude vio_result.msg.html

(cl:defclass <vio_result> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose_id
    :reader pose_id
    :initarg :pose_id
    :type cl:integer
    :initform 0)
   (time_stamp_ms
    :reader time_stamp_ms
    :initarg :time_stamp_ms
    :type cl:integer
    :initform 0)
   (flag
    :reader flag
    :initarg :flag
    :type cl:fixnum
    :initform 0)
   (p_wi
    :reader p_wi
    :initarg :p_wi
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (v_wi
    :reader v_wi
    :initarg :v_wi
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (q_wi
    :reader q_wi
    :initarg :q_wi
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (q_ic
    :reader q_ic
    :initarg :q_ic
    :type (cl:vector geometry_msgs-msg:Quaternion)
   :initform (cl:make-array 6 :element-type 'geometry_msgs-msg:Quaternion :initial-element (cl:make-instance 'geometry_msgs-msg:Quaternion)))
   (p_ic
    :reader p_ic
    :initarg :p_ic
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 6 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3)))
   (vps_from_vio
    :reader vps_from_vio
    :initarg :vps_from_vio
    :type cl:float
    :initform 0.0))
)

(cl:defclass vio_result (<vio_result>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vio_result>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vio_result)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<vio_result> is deprecated: use quadrotor_msgs-msg:vio_result instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <vio_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose_id-val :lambda-list '(m))
(cl:defmethod pose_id-val ((m <vio_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:pose_id-val is deprecated.  Use quadrotor_msgs-msg:pose_id instead.")
  (pose_id m))

(cl:ensure-generic-function 'time_stamp_ms-val :lambda-list '(m))
(cl:defmethod time_stamp_ms-val ((m <vio_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:time_stamp_ms-val is deprecated.  Use quadrotor_msgs-msg:time_stamp_ms instead.")
  (time_stamp_ms m))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <vio_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:flag-val is deprecated.  Use quadrotor_msgs-msg:flag instead.")
  (flag m))

(cl:ensure-generic-function 'p_wi-val :lambda-list '(m))
(cl:defmethod p_wi-val ((m <vio_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:p_wi-val is deprecated.  Use quadrotor_msgs-msg:p_wi instead.")
  (p_wi m))

(cl:ensure-generic-function 'v_wi-val :lambda-list '(m))
(cl:defmethod v_wi-val ((m <vio_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:v_wi-val is deprecated.  Use quadrotor_msgs-msg:v_wi instead.")
  (v_wi m))

(cl:ensure-generic-function 'q_wi-val :lambda-list '(m))
(cl:defmethod q_wi-val ((m <vio_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:q_wi-val is deprecated.  Use quadrotor_msgs-msg:q_wi instead.")
  (q_wi m))

(cl:ensure-generic-function 'q_ic-val :lambda-list '(m))
(cl:defmethod q_ic-val ((m <vio_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:q_ic-val is deprecated.  Use quadrotor_msgs-msg:q_ic instead.")
  (q_ic m))

(cl:ensure-generic-function 'p_ic-val :lambda-list '(m))
(cl:defmethod p_ic-val ((m <vio_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:p_ic-val is deprecated.  Use quadrotor_msgs-msg:p_ic instead.")
  (p_ic m))

(cl:ensure-generic-function 'vps_from_vio-val :lambda-list '(m))
(cl:defmethod vps_from_vio-val ((m <vio_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:vps_from_vio-val is deprecated.  Use quadrotor_msgs-msg:vps_from_vio instead.")
  (vps_from_vio m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vio_result>) ostream)
  "Serializes a message object of type '<vio_result>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pose_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pose_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pose_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pose_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_stamp_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time_stamp_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time_stamp_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time_stamp_ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flag)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p_wi) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'v_wi) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'q_wi) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'q_ic))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'p_ic))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vps_from_vio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vio_result>) istream)
  "Deserializes a message object of type '<vio_result>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pose_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pose_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pose_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pose_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_stamp_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time_stamp_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time_stamp_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time_stamp_ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flag)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p_wi) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'v_wi) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'q_wi) istream)
  (cl:setf (cl:slot-value msg 'q_ic) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'q_ic)))
    (cl:dotimes (i 6)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Quaternion))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'p_ic) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'p_ic)))
    (cl:dotimes (i 6)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vps_from_vio) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vio_result>)))
  "Returns string type for a message object of type '<vio_result>"
  "quadrotor_msgs/vio_result")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vio_result)))
  "Returns string type for a message object of type 'vio_result"
  "quadrotor_msgs/vio_result")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vio_result>)))
  "Returns md5sum for a message object of type '<vio_result>"
  "324b3402a89081d3fe2b8181d4ced579")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vio_result)))
  "Returns md5sum for a message object of type 'vio_result"
  "324b3402a89081d3fe2b8181d4ced579")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vio_result>)))
  "Returns full string definition for message of type '<vio_result>"
  (cl:format cl:nil "std_msgs/Header header~%uint32 pose_id~%uint32 time_stamp_ms~%uint8 flag~%geometry_msgs/Vector3 p_wi~%geometry_msgs/Vector3 v_wi~%geometry_msgs/Quaternion q_wi~%geometry_msgs/Quaternion[6] q_ic~%geometry_msgs/Vector3[6] p_ic~%float32 vps_from_vio~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vio_result)))
  "Returns full string definition for message of type 'vio_result"
  (cl:format cl:nil "std_msgs/Header header~%uint32 pose_id~%uint32 time_stamp_ms~%uint8 flag~%geometry_msgs/Vector3 p_wi~%geometry_msgs/Vector3 v_wi~%geometry_msgs/Quaternion q_wi~%geometry_msgs/Quaternion[6] q_ic~%geometry_msgs/Vector3[6] p_ic~%float32 vps_from_vio~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vio_result>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p_wi))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'v_wi))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'q_wi))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'q_ic) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'p_ic) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vio_result>))
  "Converts a ROS message object to a list"
  (cl:list 'vio_result
    (cl:cons ':header (header msg))
    (cl:cons ':pose_id (pose_id msg))
    (cl:cons ':time_stamp_ms (time_stamp_ms msg))
    (cl:cons ':flag (flag msg))
    (cl:cons ':p_wi (p_wi msg))
    (cl:cons ':v_wi (v_wi msg))
    (cl:cons ':q_wi (q_wi msg))
    (cl:cons ':q_ic (q_ic msg))
    (cl:cons ':p_ic (p_ic msg))
    (cl:cons ':vps_from_vio (vps_from_vio msg))
))
