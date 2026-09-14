; Auto-generated. Do not edit!


(cl:in-package local_sensing_node-msg)


;//! \htmlinclude DynamicObstacleState.msg.html

(cl:defclass <DynamicObstacleState> (roslisp-msg-protocol:ros-message)
  ((obstacle_id
    :reader obstacle_id
    :initarg :obstacle_id
    :type cl:integer
    :initform 0)
   (is_dynamic
    :reader is_dynamic
    :initarg :is_dynamic
    :type cl:boolean
    :initform cl:nil)
   (geometry_type
    :reader geometry_type
    :initarg :geometry_type
    :type cl:string
    :initform "")
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (size
    :reader size
    :initarg :size
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass DynamicObstacleState (<DynamicObstacleState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DynamicObstacleState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DynamicObstacleState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name local_sensing_node-msg:<DynamicObstacleState> is deprecated: use local_sensing_node-msg:DynamicObstacleState instead.")))

(cl:ensure-generic-function 'obstacle_id-val :lambda-list '(m))
(cl:defmethod obstacle_id-val ((m <DynamicObstacleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_sensing_node-msg:obstacle_id-val is deprecated.  Use local_sensing_node-msg:obstacle_id instead.")
  (obstacle_id m))

(cl:ensure-generic-function 'is_dynamic-val :lambda-list '(m))
(cl:defmethod is_dynamic-val ((m <DynamicObstacleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_sensing_node-msg:is_dynamic-val is deprecated.  Use local_sensing_node-msg:is_dynamic instead.")
  (is_dynamic m))

(cl:ensure-generic-function 'geometry_type-val :lambda-list '(m))
(cl:defmethod geometry_type-val ((m <DynamicObstacleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_sensing_node-msg:geometry_type-val is deprecated.  Use local_sensing_node-msg:geometry_type instead.")
  (geometry_type m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <DynamicObstacleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_sensing_node-msg:position-val is deprecated.  Use local_sensing_node-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <DynamicObstacleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_sensing_node-msg:velocity-val is deprecated.  Use local_sensing_node-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <DynamicObstacleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_sensing_node-msg:size-val is deprecated.  Use local_sensing_node-msg:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DynamicObstacleState>) ostream)
  "Serializes a message object of type '<DynamicObstacleState>"
  (cl:let* ((signed (cl:slot-value msg 'obstacle_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_dynamic) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'geometry_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'geometry_type))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'size) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DynamicObstacleState>) istream)
  "Deserializes a message object of type '<DynamicObstacleState>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'obstacle_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'is_dynamic) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'geometry_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'geometry_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'size) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DynamicObstacleState>)))
  "Returns string type for a message object of type '<DynamicObstacleState>"
  "local_sensing_node/DynamicObstacleState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DynamicObstacleState)))
  "Returns string type for a message object of type 'DynamicObstacleState"
  "local_sensing_node/DynamicObstacleState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DynamicObstacleState>)))
  "Returns md5sum for a message object of type '<DynamicObstacleState>"
  "d1cd6ca51f05588daa1bdc717cdb87ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DynamicObstacleState)))
  "Returns md5sum for a message object of type 'DynamicObstacleState"
  "d1cd6ca51f05588daa1bdc717cdb87ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DynamicObstacleState>)))
  "Returns full string definition for message of type '<DynamicObstacleState>"
  (cl:format cl:nil "int32 obstacle_id~%bool is_dynamic~%string geometry_type~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 size~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DynamicObstacleState)))
  "Returns full string definition for message of type 'DynamicObstacleState"
  (cl:format cl:nil "int32 obstacle_id~%bool is_dynamic~%string geometry_type~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 size~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DynamicObstacleState>))
  (cl:+ 0
     4
     1
     4 (cl:length (cl:slot-value msg 'geometry_type))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'size))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DynamicObstacleState>))
  "Converts a ROS message object to a list"
  (cl:list 'DynamicObstacleState
    (cl:cons ':obstacle_id (obstacle_id msg))
    (cl:cons ':is_dynamic (is_dynamic msg))
    (cl:cons ':geometry_type (geometry_type msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':size (size msg))
))
