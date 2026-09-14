;; Auto-generated. Do not edit!


(when (boundp 'coni_mpc::LocalBarrier)
  (if (not (find-package "CONI_MPC"))
    (make-package "CONI_MPC"))
  (shadow 'LocalBarrier (find-package "CONI_MPC")))
(unless (find-package "CONI_MPC::LOCALBARRIER")
  (make-package "CONI_MPC::LOCALBARRIER"))

(in-package "ROS")
;;//! \htmlinclude LocalBarrier.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass coni_mpc::LocalBarrier
  :super ros::object
  :slots (_header _query_center_N _A_row_major _b _c _rmse _max_abs_error _condition_number _rank _min_singular_value _max_singular_value _query_count _valid ))

(defmethod coni_mpc::LocalBarrier
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:query_center_N __query_center_N) (instance geometry_msgs::Point :init))
    ((:A_row_major __A_row_major) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:b __b) (instance geometry_msgs::Vector3 :init))
    ((:c __c) 0.0)
    ((:rmse __rmse) 0.0)
    ((:max_abs_error __max_abs_error) 0.0)
    ((:condition_number __condition_number) 0.0)
    ((:rank __rank) 0)
    ((:min_singular_value __min_singular_value) 0.0)
    ((:max_singular_value __max_singular_value) 0.0)
    ((:query_count __query_count) 0)
    ((:valid __valid) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _query_center_N __query_center_N)
   (setq _A_row_major __A_row_major)
   (setq _b __b)
   (setq _c (float __c))
   (setq _rmse (float __rmse))
   (setq _max_abs_error (float __max_abs_error))
   (setq _condition_number (float __condition_number))
   (setq _rank (round __rank))
   (setq _min_singular_value (float __min_singular_value))
   (setq _max_singular_value (float __max_singular_value))
   (setq _query_count (round __query_count))
   (setq _valid __valid)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:query_center_N
   (&rest __query_center_N)
   (if (keywordp (car __query_center_N))
       (send* _query_center_N __query_center_N)
     (progn
       (if __query_center_N (setq _query_center_N (car __query_center_N)))
       _query_center_N)))
  (:A_row_major
   (&optional __A_row_major)
   (if __A_row_major (setq _A_row_major __A_row_major)) _A_row_major)
  (:b
   (&rest __b)
   (if (keywordp (car __b))
       (send* _b __b)
     (progn
       (if __b (setq _b (car __b)))
       _b)))
  (:c
   (&optional __c)
   (if __c (setq _c __c)) _c)
  (:rmse
   (&optional __rmse)
   (if __rmse (setq _rmse __rmse)) _rmse)
  (:max_abs_error
   (&optional __max_abs_error)
   (if __max_abs_error (setq _max_abs_error __max_abs_error)) _max_abs_error)
  (:condition_number
   (&optional __condition_number)
   (if __condition_number (setq _condition_number __condition_number)) _condition_number)
  (:rank
   (&optional __rank)
   (if __rank (setq _rank __rank)) _rank)
  (:min_singular_value
   (&optional __min_singular_value)
   (if __min_singular_value (setq _min_singular_value __min_singular_value)) _min_singular_value)
  (:max_singular_value
   (&optional __max_singular_value)
   (if __max_singular_value (setq _max_singular_value __max_singular_value)) _max_singular_value)
  (:query_count
   (&optional __query_count)
   (if __query_count (setq _query_count __query_count)) _query_count)
  (:valid
   (&optional (__valid :null))
   (if (not (eq __valid :null)) (setq _valid __valid)) _valid)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _query_center_N
    (send _query_center_N :serialization-length)
    ;; float64[9] _A_row_major
    (* 8    9)
    ;; geometry_msgs/Vector3 _b
    (send _b :serialization-length)
    ;; float64 _c
    8
    ;; float64 _rmse
    8
    ;; float64 _max_abs_error
    8
    ;; float64 _condition_number
    8
    ;; int32 _rank
    4
    ;; float64 _min_singular_value
    8
    ;; float64 _max_singular_value
    8
    ;; uint32 _query_count
    4
    ;; bool _valid
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _query_center_N
       (send _query_center_N :serialize s)
     ;; float64[9] _A_row_major
     (dotimes (i 9)
       (sys::poke (elt _A_row_major i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; geometry_msgs/Vector3 _b
       (send _b :serialize s)
     ;; float64 _c
       (sys::poke _c (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rmse
       (sys::poke _rmse (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_abs_error
       (sys::poke _max_abs_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _condition_number
       (sys::poke _condition_number (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _rank
       (write-long _rank s)
     ;; float64 _min_singular_value
       (sys::poke _min_singular_value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_singular_value
       (sys::poke _max_singular_value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint32 _query_count
       (write-long _query_count s)
     ;; bool _valid
       (if _valid (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _query_center_N
     (send _query_center_N :deserialize buf ptr-) (incf ptr- (send _query_center_N :serialization-length))
   ;; float64[9] _A_row_major
   (dotimes (i (length _A_row_major))
     (setf (elt _A_row_major i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; geometry_msgs/Vector3 _b
     (send _b :deserialize buf ptr-) (incf ptr- (send _b :serialization-length))
   ;; float64 _c
     (setq _c (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rmse
     (setq _rmse (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_abs_error
     (setq _max_abs_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _condition_number
     (setq _condition_number (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _rank
     (setq _rank (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _min_singular_value
     (setq _min_singular_value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_singular_value
     (setq _max_singular_value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint32 _query_count
     (setq _query_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _valid
     (setq _valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get coni_mpc::LocalBarrier :md5sum-) "582aa024af4e145e9928e13dd367c1d7")
(setf (get coni_mpc::LocalBarrier :datatype-) "coni_mpc/LocalBarrier")
(setf (get coni_mpc::LocalBarrier :definition-)
      "std_msgs/Header header
geometry_msgs/Point query_center_N
float64[9] A_row_major
geometry_msgs/Vector3 b
float64 c
float64 rmse
float64 max_abs_error
float64 condition_number
int32 rank
float64 min_singular_value
float64 max_singular_value
uint32 query_count
bool valid

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :coni_mpc/LocalBarrier "582aa024af4e145e9928e13dd367c1d7")


