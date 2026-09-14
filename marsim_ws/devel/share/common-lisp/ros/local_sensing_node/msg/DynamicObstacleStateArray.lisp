; Auto-generated. Do not edit!


(cl:in-package local_sensing_node-msg)


;//! \htmlinclude DynamicObstacleStateArray.msg.html

(cl:defclass <DynamicObstacleStateArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obstacles
    :reader obstacles
    :initarg :obstacles
    :type (cl:vector local_sensing_node-msg:DynamicObstacleState)
   :initform (cl:make-array 0 :element-type 'local_sensing_node-msg:DynamicObstacleState :initial-element (cl:make-instance 'local_sensing_node-msg:DynamicObstacleState))))
)

(cl:defclass DynamicObstacleStateArray (<DynamicObstacleStateArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DynamicObstacleStateArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DynamicObstacleStateArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name local_sensing_node-msg:<DynamicObstacleStateArray> is deprecated: use local_sensing_node-msg:DynamicObstacleStateArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DynamicObstacleStateArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_sensing_node-msg:header-val is deprecated.  Use local_sensing_node-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'obstacles-val :lambda-list '(m))
(cl:defmethod obstacles-val ((m <DynamicObstacleStateArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader local_sensing_node-msg:obstacles-val is deprecated.  Use local_sensing_node-msg:obstacles instead.")
  (obstacles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DynamicObstacleStateArray>) ostream)
  "Serializes a message object of type '<DynamicObstacleStateArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DynamicObstacleStateArray>) istream)
  "Deserializes a message object of type '<DynamicObstacleStateArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'local_sensing_node-msg:DynamicObstacleState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DynamicObstacleStateArray>)))
  "Returns string type for a message object of type '<DynamicObstacleStateArray>"
  "local_sensing_node/DynamicObstacleStateArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DynamicObstacleStateArray)))
  "Returns string type for a message object of type 'DynamicObstacleStateArray"
  "local_sensing_node/DynamicObstacleStateArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DynamicObstacleStateArray>)))
  "Returns md5sum for a message object of type '<DynamicObstacleStateArray>"
  "2411b68bc8e07baa293691aaa8841ee6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DynamicObstacleStateArray)))
  "Returns md5sum for a message object of type 'DynamicObstacleStateArray"
  "2411b68bc8e07baa293691aaa8841ee6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DynamicObstacleStateArray>)))
  "Returns full string definition for message of type '<DynamicObstacleStateArray>"
  (cl:format cl:nil "std_msgs/Header header~%local_sensing_node/DynamicObstacleState[] obstacles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: local_sensing_node/DynamicObstacleState~%int32 obstacle_id~%bool is_dynamic~%string geometry_type~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 size~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DynamicObstacleStateArray)))
  "Returns full string definition for message of type 'DynamicObstacleStateArray"
  (cl:format cl:nil "std_msgs/Header header~%local_sensing_node/DynamicObstacleState[] obstacles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: local_sensing_node/DynamicObstacleState~%int32 obstacle_id~%bool is_dynamic~%string geometry_type~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 size~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DynamicObstacleStateArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DynamicObstacleStateArray>))
  "Converts a ROS message object to a list"
  (cl:list 'DynamicObstacleStateArray
    (cl:cons ':header (header msg))
    (cl:cons ':obstacles (obstacles msg))
))
