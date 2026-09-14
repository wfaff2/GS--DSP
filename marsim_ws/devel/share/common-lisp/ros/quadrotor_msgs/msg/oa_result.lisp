; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude oa_result.msg.html

(cl:defclass <oa_result> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (direction
    :reader direction
    :initarg :direction
    :type cl:integer
    :initform 0)
   (reserved
    :reader reserved
    :initarg :reserved
    :type cl:integer
    :initform 0)
   (tof_dist
    :reader tof_dist
    :initarg :tof_dist
    :type cl:float
    :initform 0.0)
   (image_index
    :reader image_index
    :initarg :image_index
    :type cl:integer
    :initform 0)
   (image_timestamp
    :reader image_timestamp
    :initarg :image_timestamp
    :type cl:integer
    :initform 0)
   (object_distance
    :reader object_distance
    :initarg :object_distance
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (stop
    :reader stop
    :initarg :stop
    :type cl:fixnum
    :initform 0)
   (failed_distance
    :reader failed_distance
    :initarg :failed_distance
    :type cl:fixnum
    :initform 0)
   (failed_distance_app
    :reader failed_distance_app
    :initarg :failed_distance_app
    :type cl:fixnum
    :initform 0)
   (speed_limit
    :reader speed_limit
    :initarg :speed_limit
    :type cl:fixnum
    :initform 0)
   (alarm
    :reader alarm
    :initarg :alarm
    :type cl:fixnum
    :initform 0)
   (object_distance_app
    :reader object_distance_app
    :initarg :object_distance_app
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (high_atti
    :reader high_atti
    :initarg :high_atti
    :type cl:integer
    :initform 0)
   (is_release_brake
    :reader is_release_brake
    :initarg :is_release_brake
    :type cl:integer
    :initform 0))
)

(cl:defclass oa_result (<oa_result>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <oa_result>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'oa_result)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<oa_result> is deprecated: use quadrotor_msgs-msg:oa_result instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:direction-val is deprecated.  Use quadrotor_msgs-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'reserved-val :lambda-list '(m))
(cl:defmethod reserved-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:reserved-val is deprecated.  Use quadrotor_msgs-msg:reserved instead.")
  (reserved m))

(cl:ensure-generic-function 'tof_dist-val :lambda-list '(m))
(cl:defmethod tof_dist-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:tof_dist-val is deprecated.  Use quadrotor_msgs-msg:tof_dist instead.")
  (tof_dist m))

(cl:ensure-generic-function 'image_index-val :lambda-list '(m))
(cl:defmethod image_index-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:image_index-val is deprecated.  Use quadrotor_msgs-msg:image_index instead.")
  (image_index m))

(cl:ensure-generic-function 'image_timestamp-val :lambda-list '(m))
(cl:defmethod image_timestamp-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:image_timestamp-val is deprecated.  Use quadrotor_msgs-msg:image_timestamp instead.")
  (image_timestamp m))

(cl:ensure-generic-function 'object_distance-val :lambda-list '(m))
(cl:defmethod object_distance-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:object_distance-val is deprecated.  Use quadrotor_msgs-msg:object_distance instead.")
  (object_distance m))

(cl:ensure-generic-function 'stop-val :lambda-list '(m))
(cl:defmethod stop-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:stop-val is deprecated.  Use quadrotor_msgs-msg:stop instead.")
  (stop m))

(cl:ensure-generic-function 'failed_distance-val :lambda-list '(m))
(cl:defmethod failed_distance-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:failed_distance-val is deprecated.  Use quadrotor_msgs-msg:failed_distance instead.")
  (failed_distance m))

(cl:ensure-generic-function 'failed_distance_app-val :lambda-list '(m))
(cl:defmethod failed_distance_app-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:failed_distance_app-val is deprecated.  Use quadrotor_msgs-msg:failed_distance_app instead.")
  (failed_distance_app m))

(cl:ensure-generic-function 'speed_limit-val :lambda-list '(m))
(cl:defmethod speed_limit-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:speed_limit-val is deprecated.  Use quadrotor_msgs-msg:speed_limit instead.")
  (speed_limit m))

(cl:ensure-generic-function 'alarm-val :lambda-list '(m))
(cl:defmethod alarm-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:alarm-val is deprecated.  Use quadrotor_msgs-msg:alarm instead.")
  (alarm m))

(cl:ensure-generic-function 'object_distance_app-val :lambda-list '(m))
(cl:defmethod object_distance_app-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:object_distance_app-val is deprecated.  Use quadrotor_msgs-msg:object_distance_app instead.")
  (object_distance_app m))

(cl:ensure-generic-function 'high_atti-val :lambda-list '(m))
(cl:defmethod high_atti-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:high_atti-val is deprecated.  Use quadrotor_msgs-msg:high_atti instead.")
  (high_atti m))

(cl:ensure-generic-function 'is_release_brake-val :lambda-list '(m))
(cl:defmethod is_release_brake-val ((m <oa_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:is_release_brake-val is deprecated.  Use quadrotor_msgs-msg:is_release_brake instead.")
  (is_release_brake m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <oa_result>) ostream)
  "Serializes a message object of type '<oa_result>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'direction)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'direction)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'direction)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'direction)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'reserved)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tof_dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'image_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_timestamp)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'object_distance))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stop)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'failed_distance)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'failed_distance_app)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed_limit)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'alarm)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'object_distance_app))
  (cl:let* ((signed (cl:slot-value msg 'high_atti)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'is_release_brake)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <oa_result>) istream)
  "Deserializes a message object of type '<oa_result>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'direction)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'direction)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'direction)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'direction)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reserved) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tof_dist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'image_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_timestamp)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'object_distance) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'object_distance)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stop)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'failed_distance)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'failed_distance_app)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed_limit)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'alarm)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'object_distance_app) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'object_distance_app)))
    (cl:dotimes (i 4)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'high_atti) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'is_release_brake) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<oa_result>)))
  "Returns string type for a message object of type '<oa_result>"
  "quadrotor_msgs/oa_result")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'oa_result)))
  "Returns string type for a message object of type 'oa_result"
  "quadrotor_msgs/oa_result")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<oa_result>)))
  "Returns md5sum for a message object of type '<oa_result>"
  "1f8843ccbd246ed329e5be927b5cd5cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'oa_result)))
  "Returns md5sum for a message object of type 'oa_result"
  "1f8843ccbd246ed329e5be927b5cd5cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<oa_result>)))
  "Returns full string definition for message of type '<oa_result>"
  (cl:format cl:nil "std_msgs/Header header~%uint32 direction~%int32 reserved~%float32 tof_dist~%int32 image_index~%uint32 image_timestamp~%int16[3] object_distance        # uint8 NUMBER_SUB_DIRECTIONS = 3, for FC~%uint8 stop~%uint8 failed_distance~%uint8 failed_distance_app~%uint8 speed_limit~%uint8 alarm~%int16[4] object_distance_app    # uint8 NUMBER_SUB_DIRECTIONS_FOR_APP = 4, for APP~%int32 high_atti~%int32 is_release_brake~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'oa_result)))
  "Returns full string definition for message of type 'oa_result"
  (cl:format cl:nil "std_msgs/Header header~%uint32 direction~%int32 reserved~%float32 tof_dist~%int32 image_index~%uint32 image_timestamp~%int16[3] object_distance        # uint8 NUMBER_SUB_DIRECTIONS = 3, for FC~%uint8 stop~%uint8 failed_distance~%uint8 failed_distance_app~%uint8 speed_limit~%uint8 alarm~%int16[4] object_distance_app    # uint8 NUMBER_SUB_DIRECTIONS_FOR_APP = 4, for APP~%int32 high_atti~%int32 is_release_brake~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <oa_result>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'object_distance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     1
     1
     1
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'object_distance_app) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <oa_result>))
  "Converts a ROS message object to a list"
  (cl:list 'oa_result
    (cl:cons ':header (header msg))
    (cl:cons ':direction (direction msg))
    (cl:cons ':reserved (reserved msg))
    (cl:cons ':tof_dist (tof_dist msg))
    (cl:cons ':image_index (image_index msg))
    (cl:cons ':image_timestamp (image_timestamp msg))
    (cl:cons ':object_distance (object_distance msg))
    (cl:cons ':stop (stop msg))
    (cl:cons ':failed_distance (failed_distance msg))
    (cl:cons ':failed_distance_app (failed_distance_app msg))
    (cl:cons ':speed_limit (speed_limit msg))
    (cl:cons ':alarm (alarm msg))
    (cl:cons ':object_distance_app (object_distance_app msg))
    (cl:cons ':high_atti (high_atti msg))
    (cl:cons ':is_release_brake (is_release_brake msg))
))
