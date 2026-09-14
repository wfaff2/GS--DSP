; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude ctrl.msg.html

(cl:defclass <ctrl> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (x_axis_ctrl
    :reader x_axis_ctrl
    :initarg :x_axis_ctrl
    :type cl:float
    :initform 0.0)
   (y_axis_ctrl
    :reader y_axis_ctrl
    :initarg :y_axis_ctrl
    :type cl:float
    :initform 0.0)
   (z_axis_ctrl
    :reader z_axis_ctrl
    :initarg :z_axis_ctrl
    :type cl:float
    :initform 0.0)
   (vertical_ctrl
    :reader vertical_ctrl
    :initarg :vertical_ctrl
    :type cl:float
    :initform 0.0)
   (x_feed_forward
    :reader x_feed_forward
    :initarg :x_feed_forward
    :type cl:float
    :initform 0.0)
   (y_feed_forward
    :reader y_feed_forward
    :initarg :y_feed_forward
    :type cl:float
    :initform 0.0)
   (flag
    :reader flag
    :initarg :flag
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ctrl (<ctrl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ctrl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ctrl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<ctrl> is deprecated: use quadrotor_msgs-msg:ctrl instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ctrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'x_axis_ctrl-val :lambda-list '(m))
(cl:defmethod x_axis_ctrl-val ((m <ctrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:x_axis_ctrl-val is deprecated.  Use quadrotor_msgs-msg:x_axis_ctrl instead.")
  (x_axis_ctrl m))

(cl:ensure-generic-function 'y_axis_ctrl-val :lambda-list '(m))
(cl:defmethod y_axis_ctrl-val ((m <ctrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:y_axis_ctrl-val is deprecated.  Use quadrotor_msgs-msg:y_axis_ctrl instead.")
  (y_axis_ctrl m))

(cl:ensure-generic-function 'z_axis_ctrl-val :lambda-list '(m))
(cl:defmethod z_axis_ctrl-val ((m <ctrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:z_axis_ctrl-val is deprecated.  Use quadrotor_msgs-msg:z_axis_ctrl instead.")
  (z_axis_ctrl m))

(cl:ensure-generic-function 'vertical_ctrl-val :lambda-list '(m))
(cl:defmethod vertical_ctrl-val ((m <ctrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:vertical_ctrl-val is deprecated.  Use quadrotor_msgs-msg:vertical_ctrl instead.")
  (vertical_ctrl m))

(cl:ensure-generic-function 'x_feed_forward-val :lambda-list '(m))
(cl:defmethod x_feed_forward-val ((m <ctrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:x_feed_forward-val is deprecated.  Use quadrotor_msgs-msg:x_feed_forward instead.")
  (x_feed_forward m))

(cl:ensure-generic-function 'y_feed_forward-val :lambda-list '(m))
(cl:defmethod y_feed_forward-val ((m <ctrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:y_feed_forward-val is deprecated.  Use quadrotor_msgs-msg:y_feed_forward instead.")
  (y_feed_forward m))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <ctrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:flag-val is deprecated.  Use quadrotor_msgs-msg:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ctrl>)))
    "Constants for message type '<ctrl>"
  '((:FLAG_TYPE_ATTI_VERT_POS . 18)
    (:FLAG_TYPE_VELO_YAW_RATE . 72)
    (:FLAG_TYPE_HORIZ_ANG_VERT_VEL_YAW_ANG . 1)
    (:FLAG_VERTICAL_THRUST_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY . 42)
    (:FLAG_VERTICAL_POSITION_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY . 26))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ctrl)))
    "Constants for message type 'ctrl"
  '((:FLAG_TYPE_ATTI_VERT_POS . 18)
    (:FLAG_TYPE_VELO_YAW_RATE . 72)
    (:FLAG_TYPE_HORIZ_ANG_VERT_VEL_YAW_ANG . 1)
    (:FLAG_VERTICAL_THRUST_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY . 42)
    (:FLAG_VERTICAL_POSITION_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY . 26))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ctrl>) ostream)
  "Serializes a message object of type '<ctrl>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_axis_ctrl))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_axis_ctrl))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z_axis_ctrl))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vertical_ctrl))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_feed_forward))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_feed_forward))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flag)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ctrl>) istream)
  "Deserializes a message object of type '<ctrl>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_axis_ctrl) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_axis_ctrl) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_axis_ctrl) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vertical_ctrl) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_feed_forward) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_feed_forward) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flag)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ctrl>)))
  "Returns string type for a message object of type '<ctrl>"
  "quadrotor_msgs/ctrl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ctrl)))
  "Returns string type for a message object of type 'ctrl"
  "quadrotor_msgs/ctrl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ctrl>)))
  "Returns md5sum for a message object of type '<ctrl>"
  "0f530cfce16387cbbfb41657d4d60cbc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ctrl)))
  "Returns md5sum for a message object of type 'ctrl"
  "0f530cfce16387cbbfb41657d4d60cbc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ctrl>)))
  "Returns full string definition for message of type '<ctrl>"
  (cl:format cl:nil "std_msgs/Header header~%float32 x_axis_ctrl~%float32 y_axis_ctrl~%float32 z_axis_ctrl~%float32 vertical_ctrl~%float32 x_feed_forward~%float32 y_feed_forward~%uint8   flag~%uint8 FLAG_TYPE_ATTI_VERT_POS = 18   # atti control~%uint8 FLAG_TYPE_VELO_YAW_RATE = 72   # velocity control~%uint8 FLAG_TYPE_HORIZ_ANG_VERT_VEL_YAW_ANG = 1~%uint8 FLAG_VERTICAL_THRUST_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY = 42~%uint8 FLAG_VERTICAL_POSITION_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY = 26~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ctrl)))
  "Returns full string definition for message of type 'ctrl"
  (cl:format cl:nil "std_msgs/Header header~%float32 x_axis_ctrl~%float32 y_axis_ctrl~%float32 z_axis_ctrl~%float32 vertical_ctrl~%float32 x_feed_forward~%float32 y_feed_forward~%uint8   flag~%uint8 FLAG_TYPE_ATTI_VERT_POS = 18   # atti control~%uint8 FLAG_TYPE_VELO_YAW_RATE = 72   # velocity control~%uint8 FLAG_TYPE_HORIZ_ANG_VERT_VEL_YAW_ANG = 1~%uint8 FLAG_VERTICAL_THRUST_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY = 42~%uint8 FLAG_VERTICAL_POSITION_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY = 26~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ctrl>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ctrl>))
  "Converts a ROS message object to a list"
  (cl:list 'ctrl
    (cl:cons ':header (header msg))
    (cl:cons ':x_axis_ctrl (x_axis_ctrl msg))
    (cl:cons ':y_axis_ctrl (y_axis_ctrl msg))
    (cl:cons ':z_axis_ctrl (z_axis_ctrl msg))
    (cl:cons ':vertical_ctrl (vertical_ctrl msg))
    (cl:cons ':x_feed_forward (x_feed_forward msg))
    (cl:cons ':y_feed_forward (y_feed_forward msg))
    (cl:cons ':flag (flag msg))
))
