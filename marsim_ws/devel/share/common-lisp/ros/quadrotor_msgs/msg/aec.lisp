; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude aec.msg.html

(cl:defclass <aec> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (sensor_pos
    :reader sensor_pos
    :initarg :sensor_pos
    :type cl:fixnum
    :initform 0)
   (shutter_val
    :reader shutter_val
    :initarg :shutter_val
    :type cl:fixnum
    :initform 0)
   (dgain_val
    :reader dgain_val
    :initarg :dgain_val
    :type cl:fixnum
    :initform 0)
   (dgain_a
    :reader dgain_a
    :initarg :dgain_a
    :type cl:fixnum
    :initform 0)
   (again_val
    :reader again_val
    :initarg :again_val
    :type cl:fixnum
    :initform 0)
   (again_a
    :reader again_a
    :initarg :again_a
    :type cl:fixnum
    :initform 0))
)

(cl:defclass aec (<aec>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <aec>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'aec)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<aec> is deprecated: use quadrotor_msgs-msg:aec instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <aec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'sensor_pos-val :lambda-list '(m))
(cl:defmethod sensor_pos-val ((m <aec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:sensor_pos-val is deprecated.  Use quadrotor_msgs-msg:sensor_pos instead.")
  (sensor_pos m))

(cl:ensure-generic-function 'shutter_val-val :lambda-list '(m))
(cl:defmethod shutter_val-val ((m <aec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:shutter_val-val is deprecated.  Use quadrotor_msgs-msg:shutter_val instead.")
  (shutter_val m))

(cl:ensure-generic-function 'dgain_val-val :lambda-list '(m))
(cl:defmethod dgain_val-val ((m <aec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:dgain_val-val is deprecated.  Use quadrotor_msgs-msg:dgain_val instead.")
  (dgain_val m))

(cl:ensure-generic-function 'dgain_a-val :lambda-list '(m))
(cl:defmethod dgain_a-val ((m <aec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:dgain_a-val is deprecated.  Use quadrotor_msgs-msg:dgain_a instead.")
  (dgain_a m))

(cl:ensure-generic-function 'again_val-val :lambda-list '(m))
(cl:defmethod again_val-val ((m <aec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:again_val-val is deprecated.  Use quadrotor_msgs-msg:again_val instead.")
  (again_val m))

(cl:ensure-generic-function 'again_a-val :lambda-list '(m))
(cl:defmethod again_a-val ((m <aec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:again_a-val is deprecated.  Use quadrotor_msgs-msg:again_a instead.")
  (again_a m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <aec>) ostream)
  "Serializes a message object of type '<aec>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'sensor_pos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'shutter_val)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dgain_val)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dgain_a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'again_val)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'again_a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <aec>) istream)
  "Deserializes a message object of type '<aec>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sensor_pos) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'shutter_val) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dgain_val) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dgain_a) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'again_val) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'again_a) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<aec>)))
  "Returns string type for a message object of type '<aec>"
  "quadrotor_msgs/aec")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'aec)))
  "Returns string type for a message object of type 'aec"
  "quadrotor_msgs/aec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<aec>)))
  "Returns md5sum for a message object of type '<aec>"
  "bb5f8b724b61c9413d80e9b092077813")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'aec)))
  "Returns md5sum for a message object of type 'aec"
  "bb5f8b724b61c9413d80e9b092077813")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<aec>)))
  "Returns full string definition for message of type '<aec>"
  (cl:format cl:nil "std_msgs/Header header~%int16 sensor_pos~%int16 shutter_val~%int16 dgain_val~%int16 dgain_a~%int16 again_val~%int16 again_a~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'aec)))
  "Returns full string definition for message of type 'aec"
  (cl:format cl:nil "std_msgs/Header header~%int16 sensor_pos~%int16 shutter_val~%int16 dgain_val~%int16 dgain_a~%int16 again_val~%int16 again_a~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <aec>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <aec>))
  "Converts a ROS message object to a list"
  (cl:list 'aec
    (cl:cons ':header (header msg))
    (cl:cons ':sensor_pos (sensor_pos msg))
    (cl:cons ':shutter_val (shutter_val msg))
    (cl:cons ':dgain_val (dgain_val msg))
    (cl:cons ':dgain_a (dgain_a msg))
    (cl:cons ':again_val (again_val msg))
    (cl:cons ':again_a (again_a msg))
))
