;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::MpcPositionCommand)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'MpcPositionCommand (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::MPCPOSITIONCOMMAND")
  (make-package "QUADROTOR_MSGS::MPCPOSITIONCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude MpcPositionCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::MpcPositionCommand
  :super ros::object
  :slots (_header _cmds _mpc_horizon ))

(defmethod quadrotor_msgs::MpcPositionCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cmds __cmds) ())
    ((:mpc_horizon __mpc_horizon) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _cmds __cmds)
   (setq _mpc_horizon (round __mpc_horizon))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:cmds
   (&rest __cmds)
   (if (keywordp (car __cmds))
       (send* _cmds __cmds)
     (progn
       (if __cmds (setq _cmds (car __cmds)))
       _cmds)))
  (:mpc_horizon
   (&optional __mpc_horizon)
   (if __mpc_horizon (setq _mpc_horizon __mpc_horizon)) _mpc_horizon)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; quadrotor_msgs/PositionCommand[] _cmds
    (apply #'+ (send-all _cmds :serialization-length)) 4
    ;; uint32 _mpc_horizon
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; quadrotor_msgs/PositionCommand[] _cmds
     (write-long (length _cmds) s)
     (dolist (elem _cmds)
       (send elem :serialize s)
       )
     ;; uint32 _mpc_horizon
       (write-long _mpc_horizon s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; quadrotor_msgs/PositionCommand[] _cmds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cmds (let (r) (dotimes (i n) (push (instance quadrotor_msgs::PositionCommand :init) r)) r))
     (dolist (elem- _cmds)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint32 _mpc_horizon
     (setq _mpc_horizon (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get quadrotor_msgs::MpcPositionCommand :md5sum-) "a72f7a7a678d7a5d422b50d1b2b4f694")
(setf (get quadrotor_msgs::MpcPositionCommand :datatype-) "quadrotor_msgs/MpcPositionCommand")
(setf (get quadrotor_msgs::MpcPositionCommand :definition-)
      "Header header
quadrotor_msgs/PositionCommand[] cmds
uint32 mpc_horizon

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
MSG: quadrotor_msgs/PositionCommand
Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 jerk
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 attitude
geometry_msgs/Vector3 thrust
float64 yaw
float64 yaw_dot
float64 vel_norm
float64 acc_norm


float64[3] kx
float64[3] kv
uint32 trajectory_id
uint8 TRAJECTORY_STATUS_EMPTY = 0
uint8 TRAJECTORY_STATUS_EMER = 2
uint8 TRAJECTORY_STATUS_READY = 1
uint8 TRAJECTORY_STATUS_COMPLETED = 3
uint8 TRAJECTROY_STATUS_ABORT = 4
uint8 TRAJECTORY_STATUS_ILLEGAL_START = 5
uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6
uint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7
uint32 ACTION_STOP                 =   8
# Its ID number will start from 1, allowing you comparing it with 0.
uint8 trajectory_flag

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



(provide :quadrotor_msgs/MpcPositionCommand "a72f7a7a678d7a5d422b50d1b2b4f694")


