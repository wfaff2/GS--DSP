; Auto-generated. Do not edit!


(cl:in-package coni_mpc-msg)


;//! \htmlinclude LocalBarrier.msg.html

(cl:defclass <LocalBarrier> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (query_center_N
    :reader query_center_N
    :initarg :query_center_N
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (A_row_major
    :reader A_row_major
    :initarg :A_row_major
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0))
   (b
    :reader b
    :initarg :b
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (c
    :reader c
    :initarg :c
    :type cl:float
    :initform 0.0)
   (rmse
    :reader rmse
    :initarg :rmse
    :type cl:float
    :initform 0.0)
   (max_abs_error
    :reader max_abs_error
    :initarg :max_abs_error
    :type cl:float
    :initform 0.0)
   (condition_number
    :reader condition_number
    :initarg :condition_number
    :type cl:float
    :initform 0.0)
   (rank
    :reader rank
    :initarg :rank
    :type cl:integer
    :initform 0)
   (min_singular_value
    :reader min_singular_value
    :initarg :min_singular_value
    :type cl:float
    :initform 0.0)
   (max_singular_value
    :reader max_singular_value
    :initarg :max_singular_value
    :type cl:float
    :initform 0.0)
   (query_count
    :reader query_count
    :initarg :query_count
    :type cl:integer
    :initform 0)
   (valid
    :reader valid
    :initarg :valid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LocalBarrier (<LocalBarrier>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocalBarrier>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocalBarrier)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coni_mpc-msg:<LocalBarrier> is deprecated: use coni_mpc-msg:LocalBarrier instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LocalBarrier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coni_mpc-msg:header-val is deprecated.  Use coni_mpc-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'query_center_N-val :lambda-list '(m))
(cl:defmethod query_center_N-val ((m <LocalBarrier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coni_mpc-msg:query_center_N-val is deprecated.  Use coni_mpc-msg:query_center_N instead.")
  (query_center_N m))

(cl:ensure-generic-function 'A_row_major-val :lambda-list '(m))
(cl:defmethod A_row_major-val ((m <LocalBarrier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coni_mpc-msg:A_row_major-val is deprecated.  Use coni_mpc-msg:A_row_major instead.")
  (A_row_major m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <LocalBarrier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coni_mpc-msg:b-val is deprecated.  Use coni_mpc-msg:b instead.")
  (b m))

(cl:ensure-generic-function 'c-val :lambda-list '(m))
(cl:defmethod c-val ((m <LocalBarrier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coni_mpc-msg:c-val is deprecated.  Use coni_mpc-msg:c instead.")
  (c m))

(cl:ensure-generic-function 'rmse-val :lambda-list '(m))
(cl:defmethod rmse-val ((m <LocalBarrier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coni_mpc-msg:rmse-val is deprecated.  Use coni_mpc-msg:rmse instead.")
  (rmse m))

(cl:ensure-generic-function 'max_abs_error-val :lambda-list '(m))
(cl:defmethod max_abs_error-val ((m <LocalBarrier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coni_mpc-msg:max_abs_error-val is deprecated.  Use coni_mpc-msg:max_abs_error instead.")
  (max_abs_error m))

(cl:ensure-generic-function 'condition_number-val :lambda-list '(m))
(cl:defmethod condition_number-val ((m <LocalBarrier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coni_mpc-msg:condition_number-val is deprecated.  Use coni_mpc-msg:condition_number instead.")
  (condition_number m))

(cl:ensure-generic-function 'rank-val :lambda-list '(m))
(cl:defmethod rank-val ((m <LocalBarrier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coni_mpc-msg:rank-val is deprecated.  Use coni_mpc-msg:rank instead.")
  (rank m))

(cl:ensure-generic-function 'min_singular_value-val :lambda-list '(m))
(cl:defmethod min_singular_value-val ((m <LocalBarrier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coni_mpc-msg:min_singular_value-val is deprecated.  Use coni_mpc-msg:min_singular_value instead.")
  (min_singular_value m))

(cl:ensure-generic-function 'max_singular_value-val :lambda-list '(m))
(cl:defmethod max_singular_value-val ((m <LocalBarrier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coni_mpc-msg:max_singular_value-val is deprecated.  Use coni_mpc-msg:max_singular_value instead.")
  (max_singular_value m))

(cl:ensure-generic-function 'query_count-val :lambda-list '(m))
(cl:defmethod query_count-val ((m <LocalBarrier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coni_mpc-msg:query_count-val is deprecated.  Use coni_mpc-msg:query_count instead.")
  (query_count m))

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <LocalBarrier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coni_mpc-msg:valid-val is deprecated.  Use coni_mpc-msg:valid instead.")
  (valid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocalBarrier>) ostream)
  "Serializes a message object of type '<LocalBarrier>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'query_center_N) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'A_row_major))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'b) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rmse))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_abs_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'condition_number))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'rank)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min_singular_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_singular_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'query_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'query_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'query_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'query_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'valid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocalBarrier>) istream)
  "Deserializes a message object of type '<LocalBarrier>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'query_center_N) istream)
  (cl:setf (cl:slot-value msg 'A_row_major) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'A_row_major)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'b) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'c) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rmse) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_abs_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'condition_number) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rank) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_singular_value) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_singular_value) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'query_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'query_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'query_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'query_count)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'valid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocalBarrier>)))
  "Returns string type for a message object of type '<LocalBarrier>"
  "coni_mpc/LocalBarrier")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocalBarrier)))
  "Returns string type for a message object of type 'LocalBarrier"
  "coni_mpc/LocalBarrier")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocalBarrier>)))
  "Returns md5sum for a message object of type '<LocalBarrier>"
  "582aa024af4e145e9928e13dd367c1d7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocalBarrier)))
  "Returns md5sum for a message object of type 'LocalBarrier"
  "582aa024af4e145e9928e13dd367c1d7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocalBarrier>)))
  "Returns full string definition for message of type '<LocalBarrier>"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Point query_center_N~%float64[9] A_row_major~%geometry_msgs/Vector3 b~%float64 c~%float64 rmse~%float64 max_abs_error~%float64 condition_number~%int32 rank~%float64 min_singular_value~%float64 max_singular_value~%uint32 query_count~%bool valid~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocalBarrier)))
  "Returns full string definition for message of type 'LocalBarrier"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Point query_center_N~%float64[9] A_row_major~%geometry_msgs/Vector3 b~%float64 c~%float64 rmse~%float64 max_abs_error~%float64 condition_number~%int32 rank~%float64 min_singular_value~%float64 max_singular_value~%uint32 query_count~%bool valid~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocalBarrier>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'query_center_N))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'A_row_major) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'b))
     8
     8
     8
     8
     4
     8
     8
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocalBarrier>))
  "Converts a ROS message object to a list"
  (cl:list 'LocalBarrier
    (cl:cons ':header (header msg))
    (cl:cons ':query_center_N (query_center_N msg))
    (cl:cons ':A_row_major (A_row_major msg))
    (cl:cons ':b (b msg))
    (cl:cons ':c (c msg))
    (cl:cons ':rmse (rmse msg))
    (cl:cons ':max_abs_error (max_abs_error msg))
    (cl:cons ':condition_number (condition_number msg))
    (cl:cons ':rank (rank msg))
    (cl:cons ':min_singular_value (min_singular_value msg))
    (cl:cons ':max_singular_value (max_singular_value msg))
    (cl:cons ':query_count (query_count msg))
    (cl:cons ':valid (valid msg))
))
