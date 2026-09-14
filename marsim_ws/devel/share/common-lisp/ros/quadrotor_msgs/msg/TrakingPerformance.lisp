; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude TrakingPerformance.msg.html

(cl:defclass <TrakingPerformance> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (fsm_state
    :reader fsm_state
    :initarg :fsm_state
    :type std_msgs-msg:Int64
    :initform (cl:make-instance 'std_msgs-msg:Int64))
   (mpc_solve_time
    :reader mpc_solve_time
    :initarg :mpc_solve_time
    :type cl:float
    :initform 0.0)
   (desire
    :reader desire
    :initarg :desire
    :type quadrotor_msgs-msg:QuadrotorState
    :initform (cl:make-instance 'quadrotor_msgs-msg:QuadrotorState))
   (feedback
    :reader feedback
    :initarg :feedback
    :type quadrotor_msgs-msg:QuadrotorState
    :initform (cl:make-instance 'quadrotor_msgs-msg:QuadrotorState))
   (error
    :reader error
    :initarg :error
    :type quadrotor_msgs-msg:QuadrotorState
    :initform (cl:make-instance 'quadrotor_msgs-msg:QuadrotorState)))
)

(cl:defclass TrakingPerformance (<TrakingPerformance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrakingPerformance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrakingPerformance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<TrakingPerformance> is deprecated: use quadrotor_msgs-msg:TrakingPerformance instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrakingPerformance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'fsm_state-val :lambda-list '(m))
(cl:defmethod fsm_state-val ((m <TrakingPerformance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:fsm_state-val is deprecated.  Use quadrotor_msgs-msg:fsm_state instead.")
  (fsm_state m))

(cl:ensure-generic-function 'mpc_solve_time-val :lambda-list '(m))
(cl:defmethod mpc_solve_time-val ((m <TrakingPerformance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:mpc_solve_time-val is deprecated.  Use quadrotor_msgs-msg:mpc_solve_time instead.")
  (mpc_solve_time m))

(cl:ensure-generic-function 'desire-val :lambda-list '(m))
(cl:defmethod desire-val ((m <TrakingPerformance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:desire-val is deprecated.  Use quadrotor_msgs-msg:desire instead.")
  (desire m))

(cl:ensure-generic-function 'feedback-val :lambda-list '(m))
(cl:defmethod feedback-val ((m <TrakingPerformance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:feedback-val is deprecated.  Use quadrotor_msgs-msg:feedback instead.")
  (feedback m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <TrakingPerformance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:error-val is deprecated.  Use quadrotor_msgs-msg:error instead.")
  (error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrakingPerformance>) ostream)
  "Serializes a message object of type '<TrakingPerformance>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'fsm_state) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mpc_solve_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desire) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'feedback) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'error) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrakingPerformance>) istream)
  "Deserializes a message object of type '<TrakingPerformance>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'fsm_state) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mpc_solve_time) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desire) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'feedback) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'error) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrakingPerformance>)))
  "Returns string type for a message object of type '<TrakingPerformance>"
  "quadrotor_msgs/TrakingPerformance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrakingPerformance)))
  "Returns string type for a message object of type 'TrakingPerformance"
  "quadrotor_msgs/TrakingPerformance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrakingPerformance>)))
  "Returns md5sum for a message object of type '<TrakingPerformance>"
  "91224dc465a6f21d706763bf5df5f87e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrakingPerformance)))
  "Returns md5sum for a message object of type 'TrakingPerformance"
  "91224dc465a6f21d706763bf5df5f87e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrakingPerformance>)))
  "Returns full string definition for message of type '<TrakingPerformance>"
  (cl:format cl:nil "# Header~%std_msgs/Header header~%# MPC finite state machine state~%std_msgs/Int64 fsm_state~%# Solving time consuming~%float64 mpc_solve_time~%~%QuadrotorState desire~%QuadrotorState feedback~%QuadrotorState error~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: quadrotor_msgs/QuadrotorState~%float64 thrust~%float64 velocity_norm~%float64 acceleration_norm~%float64 jerk_norm~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%geometry_msgs/Vector3 jerk~%geometry_msgs/Vector3 snap~%geometry_msgs/Vector3 attitude~%geometry_msgs/Vector3 angular_velocity~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrakingPerformance)))
  "Returns full string definition for message of type 'TrakingPerformance"
  (cl:format cl:nil "# Header~%std_msgs/Header header~%# MPC finite state machine state~%std_msgs/Int64 fsm_state~%# Solving time consuming~%float64 mpc_solve_time~%~%QuadrotorState desire~%QuadrotorState feedback~%QuadrotorState error~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: quadrotor_msgs/QuadrotorState~%float64 thrust~%float64 velocity_norm~%float64 acceleration_norm~%float64 jerk_norm~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%geometry_msgs/Vector3 jerk~%geometry_msgs/Vector3 snap~%geometry_msgs/Vector3 attitude~%geometry_msgs/Vector3 angular_velocity~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrakingPerformance>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'fsm_state))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desire))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'feedback))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'error))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrakingPerformance>))
  "Converts a ROS message object to a list"
  (cl:list 'TrakingPerformance
    (cl:cons ':header (header msg))
    (cl:cons ':fsm_state (fsm_state msg))
    (cl:cons ':mpc_solve_time (mpc_solve_time msg))
    (cl:cons ':desire (desire msg))
    (cl:cons ':feedback (feedback msg))
    (cl:cons ':error (error msg))
))
