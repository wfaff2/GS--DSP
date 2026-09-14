; Auto-generated. Do not edit!


(cl:in-package onboard_detector-msg)


;//! \htmlinclude TrackedObstacle.msg.html

(cl:defclass <TrackedObstacle> (roslisp-msg-protocol:ros-message)
  ((slot_id
    :reader slot_id
    :initarg :slot_id
    :type cl:integer
    :initform 0)
   (is_dynamic
    :reader is_dynamic
    :initarg :is_dynamic
    :type cl:boolean
    :initform cl:nil)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
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

(cl:defclass TrackedObstacle (<TrackedObstacle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackedObstacle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackedObstacle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name onboard_detector-msg:<TrackedObstacle> is deprecated: use onboard_detector-msg:TrackedObstacle instead.")))

(cl:ensure-generic-function 'slot_id-val :lambda-list '(m))
(cl:defmethod slot_id-val ((m <TrackedObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader onboard_detector-msg:slot_id-val is deprecated.  Use onboard_detector-msg:slot_id instead.")
  (slot_id m))

(cl:ensure-generic-function 'is_dynamic-val :lambda-list '(m))
(cl:defmethod is_dynamic-val ((m <TrackedObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader onboard_detector-msg:is_dynamic-val is deprecated.  Use onboard_detector-msg:is_dynamic instead.")
  (is_dynamic m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <TrackedObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader onboard_detector-msg:position-val is deprecated.  Use onboard_detector-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <TrackedObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader onboard_detector-msg:velocity-val is deprecated.  Use onboard_detector-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <TrackedObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader onboard_detector-msg:size-val is deprecated.  Use onboard_detector-msg:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackedObstacle>) ostream)
  "Serializes a message object of type '<TrackedObstacle>"
  (cl:let* ((signed (cl:slot-value msg 'slot_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_dynamic) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'size) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackedObstacle>) istream)
  "Deserializes a message object of type '<TrackedObstacle>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'slot_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'is_dynamic) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'size) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackedObstacle>)))
  "Returns string type for a message object of type '<TrackedObstacle>"
  "onboard_detector/TrackedObstacle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackedObstacle)))
  "Returns string type for a message object of type 'TrackedObstacle"
  "onboard_detector/TrackedObstacle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackedObstacle>)))
  "Returns md5sum for a message object of type '<TrackedObstacle>"
  "50d60af2ec9911de5b888d1721cba72d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackedObstacle)))
  "Returns md5sum for a message object of type 'TrackedObstacle"
  "50d60af2ec9911de5b888d1721cba72d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackedObstacle>)))
  "Returns full string definition for message of type '<TrackedObstacle>"
  (cl:format cl:nil "int32 slot_id~%bool is_dynamic~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 size~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackedObstacle)))
  "Returns full string definition for message of type 'TrackedObstacle"
  (cl:format cl:nil "int32 slot_id~%bool is_dynamic~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 size~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackedObstacle>))
  (cl:+ 0
     4
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'size))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackedObstacle>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackedObstacle
    (cl:cons ':slot_id (slot_id msg))
    (cl:cons ':is_dynamic (is_dynamic msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':size (size msg))
))
