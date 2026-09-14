;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::MincoTrajectory)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'MincoTrajectory (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::MINCOTRAJECTORY")
  (make-package "QUADROTOR_MSGS::MINCOTRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude MincoTrajectory.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*ACTION_ADD*" (find-package "QUADROTOR_MSGS::MINCOTRAJECTORY"))
(shadow '*ACTION_ADD* (find-package "QUADROTOR_MSGS::MINCOTRAJECTORY"))
(defconstant quadrotor_msgs::MincoTrajectory::*ACTION_ADD* 1)
(intern "*ACTION_ABORT*" (find-package "QUADROTOR_MSGS::MINCOTRAJECTORY"))
(shadow '*ACTION_ABORT* (find-package "QUADROTOR_MSGS::MINCOTRAJECTORY"))
(defconstant quadrotor_msgs::MincoTrajectory::*ACTION_ABORT* 2)
(intern "*ACTION_WARN_START*" (find-package "QUADROTOR_MSGS::MINCOTRAJECTORY"))
(shadow '*ACTION_WARN_START* (find-package "QUADROTOR_MSGS::MINCOTRAJECTORY"))
(defconstant quadrotor_msgs::MincoTrajectory::*ACTION_WARN_START* 3)
(intern "*ACTION_WARN_FINAL*" (find-package "QUADROTOR_MSGS::MINCOTRAJECTORY"))
(shadow '*ACTION_WARN_FINAL* (find-package "QUADROTOR_MSGS::MINCOTRAJECTORY"))
(defconstant quadrotor_msgs::MincoTrajectory::*ACTION_WARN_FINAL* 4)
(intern "*ACTION_WARN_IMPOSSIBLE*" (find-package "QUADROTOR_MSGS::MINCOTRAJECTORY"))
(shadow '*ACTION_WARN_IMPOSSIBLE* (find-package "QUADROTOR_MSGS::MINCOTRAJECTORY"))
(defconstant quadrotor_msgs::MincoTrajectory::*ACTION_WARN_IMPOSSIBLE* 5)

(defun quadrotor_msgs::MincoTrajectory-to-symbol (const)
  (cond
        ((= const 1) 'quadrotor_msgs::MincoTrajectory::*ACTION_ADD*)
        ((= const 2) 'quadrotor_msgs::MincoTrajectory::*ACTION_ABORT*)
        ((= const 3) 'quadrotor_msgs::MincoTrajectory::*ACTION_WARN_START*)
        ((= const 4) 'quadrotor_msgs::MincoTrajectory::*ACTION_WARN_FINAL*)
        ((= const 5) 'quadrotor_msgs::MincoTrajectory::*ACTION_WARN_IMPOSSIBLE*)
        (t nil)))

(defclass quadrotor_msgs::MincoTrajectory
  :super ros::object
  :slots (_header _trajectory_id _action _num_order _num_segment _start_yaw _final_yaw _init_state _final_state _waypoints_x _waypoints_y _waypoints_z _time _mag_coeff _order _start_time _local_start_t _occ_state _debug_info ))

(defmethod quadrotor_msgs::MincoTrajectory
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:trajectory_id __trajectory_id) 0)
    ((:action __action) 0)
    ((:num_order __num_order) 0)
    ((:num_segment __num_segment) 0)
    ((:start_yaw __start_yaw) 0.0)
    ((:final_yaw __final_yaw) 0.0)
    ((:init_state __init_state) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:final_state __final_state) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:waypoints_x __waypoints_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:waypoints_y __waypoints_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:waypoints_z __waypoints_z) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:time __time) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:mag_coeff __mag_coeff) 0.0)
    ((:order __order) (make-array 0 :initial-element 0 :element-type :integer))
    ((:start_time __start_time) (instance ros::time :init))
    ((:local_start_t __local_start_t) 0.0)
    ((:occ_state __occ_state) 0)
    ((:debug_info __debug_info) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _trajectory_id (round __trajectory_id))
   (setq _action (round __action))
   (setq _num_order (round __num_order))
   (setq _num_segment (round __num_segment))
   (setq _start_yaw (float __start_yaw))
   (setq _final_yaw (float __final_yaw))
   (setq _init_state __init_state)
   (setq _final_state __final_state)
   (setq _waypoints_x __waypoints_x)
   (setq _waypoints_y __waypoints_y)
   (setq _waypoints_z __waypoints_z)
   (setq _time __time)
   (setq _mag_coeff (float __mag_coeff))
   (setq _order __order)
   (setq _start_time __start_time)
   (setq _local_start_t (float __local_start_t))
   (setq _occ_state (round __occ_state))
   (setq _debug_info (string __debug_info))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:trajectory_id
   (&optional __trajectory_id)
   (if __trajectory_id (setq _trajectory_id __trajectory_id)) _trajectory_id)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:num_order
   (&optional __num_order)
   (if __num_order (setq _num_order __num_order)) _num_order)
  (:num_segment
   (&optional __num_segment)
   (if __num_segment (setq _num_segment __num_segment)) _num_segment)
  (:start_yaw
   (&optional __start_yaw)
   (if __start_yaw (setq _start_yaw __start_yaw)) _start_yaw)
  (:final_yaw
   (&optional __final_yaw)
   (if __final_yaw (setq _final_yaw __final_yaw)) _final_yaw)
  (:init_state
   (&optional __init_state)
   (if __init_state (setq _init_state __init_state)) _init_state)
  (:final_state
   (&optional __final_state)
   (if __final_state (setq _final_state __final_state)) _final_state)
  (:waypoints_x
   (&optional __waypoints_x)
   (if __waypoints_x (setq _waypoints_x __waypoints_x)) _waypoints_x)
  (:waypoints_y
   (&optional __waypoints_y)
   (if __waypoints_y (setq _waypoints_y __waypoints_y)) _waypoints_y)
  (:waypoints_z
   (&optional __waypoints_z)
   (if __waypoints_z (setq _waypoints_z __waypoints_z)) _waypoints_z)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:mag_coeff
   (&optional __mag_coeff)
   (if __mag_coeff (setq _mag_coeff __mag_coeff)) _mag_coeff)
  (:order
   (&optional __order)
   (if __order (setq _order __order)) _order)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:local_start_t
   (&optional __local_start_t)
   (if __local_start_t (setq _local_start_t __local_start_t)) _local_start_t)
  (:occ_state
   (&optional __occ_state)
   (if __occ_state (setq _occ_state __occ_state)) _occ_state)
  (:debug_info
   (&optional __debug_info)
   (if __debug_info (setq _debug_info __debug_info)) _debug_info)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _trajectory_id
    4
    ;; uint32 _action
    4
    ;; uint32 _num_order
    4
    ;; uint32 _num_segment
    4
    ;; float64 _start_yaw
    8
    ;; float64 _final_yaw
    8
    ;; float64[12] _init_state
    (* 8    12)
    ;; float64[12] _final_state
    (* 8    12)
    ;; float64[] _waypoints_x
    (* 8    (length _waypoints_x)) 4
    ;; float64[] _waypoints_y
    (* 8    (length _waypoints_y)) 4
    ;; float64[] _waypoints_z
    (* 8    (length _waypoints_z)) 4
    ;; float64[] _time
    (* 8    (length _time)) 4
    ;; float64 _mag_coeff
    8
    ;; uint32[] _order
    (* 4    (length _order)) 4
    ;; time _start_time
    8
    ;; float64 _local_start_t
    8
    ;; uint32 _occ_state
    4
    ;; string _debug_info
    4 (length _debug_info)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _trajectory_id
       (write-long _trajectory_id s)
     ;; uint32 _action
       (write-long _action s)
     ;; uint32 _num_order
       (write-long _num_order s)
     ;; uint32 _num_segment
       (write-long _num_segment s)
     ;; float64 _start_yaw
       (sys::poke _start_yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _final_yaw
       (sys::poke _final_yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[12] _init_state
     (dotimes (i 12)
       (sys::poke (elt _init_state i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[12] _final_state
     (dotimes (i 12)
       (sys::poke (elt _final_state i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _waypoints_x
     (write-long (length _waypoints_x) s)
     (dotimes (i (length _waypoints_x))
       (sys::poke (elt _waypoints_x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _waypoints_y
     (write-long (length _waypoints_y) s)
     (dotimes (i (length _waypoints_y))
       (sys::poke (elt _waypoints_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _waypoints_z
     (write-long (length _waypoints_z) s)
     (dotimes (i (length _waypoints_z))
       (sys::poke (elt _waypoints_z i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _time
     (write-long (length _time) s)
     (dotimes (i (length _time))
       (sys::poke (elt _time i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _mag_coeff
       (sys::poke _mag_coeff (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint32[] _order
     (write-long (length _order) s)
     (dotimes (i (length _order))
       (write-long (elt _order i) s)
       )
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
     ;; float64 _local_start_t
       (sys::poke _local_start_t (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint32 _occ_state
       (write-long _occ_state s)
     ;; string _debug_info
       (write-long (length _debug_info) s) (princ _debug_info s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _trajectory_id
     (setq _trajectory_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _action
     (setq _action (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _num_order
     (setq _num_order (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _num_segment
     (setq _num_segment (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _start_yaw
     (setq _start_yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _final_yaw
     (setq _final_yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[12] _init_state
   (dotimes (i (length _init_state))
     (setf (elt _init_state i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[12] _final_state
   (dotimes (i (length _final_state))
     (setf (elt _final_state i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[] _waypoints_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _waypoints_x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _waypoints_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _waypoints_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _waypoints_z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints_z (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _waypoints_z i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _time
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _time (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _time i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _mag_coeff
     (setq _mag_coeff (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint32[] _order
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _order (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _order i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _local_start_t
     (setq _local_start_t (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint32 _occ_state
     (setq _occ_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _debug_info
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _debug_info (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get quadrotor_msgs::MincoTrajectory :md5sum-) "00d62cdfef43fbf8db0e1a3a7f618be6")
(setf (get quadrotor_msgs::MincoTrajectory :datatype-) "quadrotor_msgs/MincoTrajectory")
(setf (get quadrotor_msgs::MincoTrajectory :definition-)
      "Header header

# the trajectory id, starts from \"1\".
uint32 trajectory_id

# the action command for trajectory server.
uint32 ACTION_ADD                  =   1
uint32 ACTION_ABORT                =   2
uint32 ACTION_WARN_START           =   3
uint32 ACTION_WARN_FINAL           =   4
uint32 ACTION_WARN_IMPOSSIBLE      =   5
uint32 action

# the order of trajectory.
uint32 num_order
uint32 num_segment

# the polynomial coecfficients of the trajectory.
float64 start_yaw
float64 final_yaw
float64[12] init_state
float64[12] final_state
float64[] waypoints_x
float64[] waypoints_y
float64[] waypoints_z
float64[] time
float64   mag_coeff
uint32[]  order
time start_time
float64 local_start_t
#the occ state of trajectory.
uint32 occ_state
string debug_info

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

")



(provide :quadrotor_msgs/MincoTrajectory "00d62cdfef43fbf8db0e1a3a7f618be6")


