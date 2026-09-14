;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::TrakingPerformance)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'TrakingPerformance (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::TRAKINGPERFORMANCE")
  (make-package "QUADROTOR_MSGS::TRAKINGPERFORMANCE"))

(in-package "ROS")
;;//! \htmlinclude TrakingPerformance.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::TrakingPerformance
  :super ros::object
  :slots (_header _fsm_state _mpc_solve_time _desire _feedback _error ))

(defmethod quadrotor_msgs::TrakingPerformance
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:fsm_state __fsm_state) (instance std_msgs::Int64 :init))
    ((:mpc_solve_time __mpc_solve_time) 0.0)
    ((:desire __desire) (instance quadrotor_msgs::QuadrotorState :init))
    ((:feedback __feedback) (instance quadrotor_msgs::QuadrotorState :init))
    ((:error __error) (instance quadrotor_msgs::QuadrotorState :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _fsm_state __fsm_state)
   (setq _mpc_solve_time (float __mpc_solve_time))
   (setq _desire __desire)
   (setq _feedback __feedback)
   (setq _error __error)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:fsm_state
   (&rest __fsm_state)
   (if (keywordp (car __fsm_state))
       (send* _fsm_state __fsm_state)
     (progn
       (if __fsm_state (setq _fsm_state (car __fsm_state)))
       _fsm_state)))
  (:mpc_solve_time
   (&optional __mpc_solve_time)
   (if __mpc_solve_time (setq _mpc_solve_time __mpc_solve_time)) _mpc_solve_time)
  (:desire
   (&rest __desire)
   (if (keywordp (car __desire))
       (send* _desire __desire)
     (progn
       (if __desire (setq _desire (car __desire)))
       _desire)))
  (:feedback
   (&rest __feedback)
   (if (keywordp (car __feedback))
       (send* _feedback __feedback)
     (progn
       (if __feedback (setq _feedback (car __feedback)))
       _feedback)))
  (:error
   (&rest __error)
   (if (keywordp (car __error))
       (send* _error __error)
     (progn
       (if __error (setq _error (car __error)))
       _error)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/Int64 _fsm_state
    (send _fsm_state :serialization-length)
    ;; float64 _mpc_solve_time
    8
    ;; quadrotor_msgs/QuadrotorState _desire
    (send _desire :serialization-length)
    ;; quadrotor_msgs/QuadrotorState _feedback
    (send _feedback :serialization-length)
    ;; quadrotor_msgs/QuadrotorState _error
    (send _error :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/Int64 _fsm_state
       (send _fsm_state :serialize s)
     ;; float64 _mpc_solve_time
       (sys::poke _mpc_solve_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; quadrotor_msgs/QuadrotorState _desire
       (send _desire :serialize s)
     ;; quadrotor_msgs/QuadrotorState _feedback
       (send _feedback :serialize s)
     ;; quadrotor_msgs/QuadrotorState _error
       (send _error :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/Int64 _fsm_state
     (send _fsm_state :deserialize buf ptr-) (incf ptr- (send _fsm_state :serialization-length))
   ;; float64 _mpc_solve_time
     (setq _mpc_solve_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; quadrotor_msgs/QuadrotorState _desire
     (send _desire :deserialize buf ptr-) (incf ptr- (send _desire :serialization-length))
   ;; quadrotor_msgs/QuadrotorState _feedback
     (send _feedback :deserialize buf ptr-) (incf ptr- (send _feedback :serialization-length))
   ;; quadrotor_msgs/QuadrotorState _error
     (send _error :deserialize buf ptr-) (incf ptr- (send _error :serialization-length))
   ;;
   self)
  )

(setf (get quadrotor_msgs::TrakingPerformance :md5sum-) "91224dc465a6f21d706763bf5df5f87e")
(setf (get quadrotor_msgs::TrakingPerformance :datatype-) "quadrotor_msgs/TrakingPerformance")
(setf (get quadrotor_msgs::TrakingPerformance :definition-)
      "# Header
std_msgs/Header header
# MPC finite state machine state
std_msgs/Int64 fsm_state
# Solving time consuming
float64 mpc_solve_time

QuadrotorState desire
QuadrotorState feedback
QuadrotorState error



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
MSG: std_msgs/Int64
int64 data
================================================================================
MSG: quadrotor_msgs/QuadrotorState
float64 thrust
float64 velocity_norm
float64 acceleration_norm
float64 jerk_norm
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 jerk
geometry_msgs/Vector3 snap
geometry_msgs/Vector3 attitude
geometry_msgs/Vector3 angular_velocity


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



(provide :quadrotor_msgs/TrakingPerformance "91224dc465a6f21d706763bf5df5f87e")


