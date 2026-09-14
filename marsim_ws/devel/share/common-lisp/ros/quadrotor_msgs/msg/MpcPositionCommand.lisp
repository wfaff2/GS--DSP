; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude MpcPositionCommand.msg.html

(cl:defclass <MpcPositionCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cmds
    :reader cmds
    :initarg :cmds
    :type (cl:vector quadrotor_msgs-msg:PositionCommand)
   :initform (cl:make-array 0 :element-type 'quadrotor_msgs-msg:PositionCommand :initial-element (cl:make-instance 'quadrotor_msgs-msg:PositionCommand)))
   (mpc_horizon
    :reader mpc_horizon
    :initarg :mpc_horizon
    :type cl:integer
    :initform 0))
)

(cl:defclass MpcPositionCommand (<MpcPositionCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MpcPositionCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MpcPositionCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<MpcPositionCommand> is deprecated: use quadrotor_msgs-msg:MpcPositionCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MpcPositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cmds-val :lambda-list '(m))
(cl:defmethod cmds-val ((m <MpcPositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:cmds-val is deprecated.  Use quadrotor_msgs-msg:cmds instead.")
  (cmds m))

(cl:ensure-generic-function 'mpc_horizon-val :lambda-list '(m))
(cl:defmethod mpc_horizon-val ((m <MpcPositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:mpc_horizon-val is deprecated.  Use quadrotor_msgs-msg:mpc_horizon instead.")
  (mpc_horizon m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MpcPositionCommand>) ostream)
  "Serializes a message object of type '<MpcPositionCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cmds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cmds))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mpc_horizon)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mpc_horizon)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mpc_horizon)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mpc_horizon)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MpcPositionCommand>) istream)
  "Deserializes a message object of type '<MpcPositionCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cmds) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cmds)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'quadrotor_msgs-msg:PositionCommand))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mpc_horizon)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mpc_horizon)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mpc_horizon)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mpc_horizon)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MpcPositionCommand>)))
  "Returns string type for a message object of type '<MpcPositionCommand>"
  "quadrotor_msgs/MpcPositionCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MpcPositionCommand)))
  "Returns string type for a message object of type 'MpcPositionCommand"
  "quadrotor_msgs/MpcPositionCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MpcPositionCommand>)))
  "Returns md5sum for a message object of type '<MpcPositionCommand>"
  "a72f7a7a678d7a5d422b50d1b2b4f694")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MpcPositionCommand)))
  "Returns md5sum for a message object of type 'MpcPositionCommand"
  "a72f7a7a678d7a5d422b50d1b2b4f694")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MpcPositionCommand>)))
  "Returns full string definition for message of type '<MpcPositionCommand>"
  (cl:format cl:nil "Header header~%quadrotor_msgs/PositionCommand[] cmds~%uint32 mpc_horizon~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: quadrotor_msgs/PositionCommand~%Header header~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%geometry_msgs/Vector3 jerk~%geometry_msgs/Vector3 angular_velocity~%geometry_msgs/Vector3 attitude~%geometry_msgs/Vector3 thrust~%float64 yaw~%float64 yaw_dot~%float64 vel_norm~%float64 acc_norm~%~%~%float64[3] kx~%float64[3] kv~%uint32 trajectory_id~%uint8 TRAJECTORY_STATUS_EMPTY = 0~%uint8 TRAJECTORY_STATUS_EMER = 2~%uint8 TRAJECTORY_STATUS_READY = 1~%uint8 TRAJECTORY_STATUS_COMPLETED = 3~%uint8 TRAJECTROY_STATUS_ABORT = 4~%uint8 TRAJECTORY_STATUS_ILLEGAL_START = 5~%uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6~%uint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7~%uint32 ACTION_STOP                 =   8~%# Its ID number will start from 1, allowing you comparing it with 0.~%uint8 trajectory_flag~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MpcPositionCommand)))
  "Returns full string definition for message of type 'MpcPositionCommand"
  (cl:format cl:nil "Header header~%quadrotor_msgs/PositionCommand[] cmds~%uint32 mpc_horizon~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: quadrotor_msgs/PositionCommand~%Header header~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%geometry_msgs/Vector3 jerk~%geometry_msgs/Vector3 angular_velocity~%geometry_msgs/Vector3 attitude~%geometry_msgs/Vector3 thrust~%float64 yaw~%float64 yaw_dot~%float64 vel_norm~%float64 acc_norm~%~%~%float64[3] kx~%float64[3] kv~%uint32 trajectory_id~%uint8 TRAJECTORY_STATUS_EMPTY = 0~%uint8 TRAJECTORY_STATUS_EMER = 2~%uint8 TRAJECTORY_STATUS_READY = 1~%uint8 TRAJECTORY_STATUS_COMPLETED = 3~%uint8 TRAJECTROY_STATUS_ABORT = 4~%uint8 TRAJECTORY_STATUS_ILLEGAL_START = 5~%uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6~%uint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7~%uint32 ACTION_STOP                 =   8~%# Its ID number will start from 1, allowing you comparing it with 0.~%uint8 trajectory_flag~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MpcPositionCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cmds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MpcPositionCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'MpcPositionCommand
    (cl:cons ':header (header msg))
    (cl:cons ':cmds (cmds msg))
    (cl:cons ':mpc_horizon (mpc_horizon msg))
))
