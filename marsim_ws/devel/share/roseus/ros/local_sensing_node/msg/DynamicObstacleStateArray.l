;; Auto-generated. Do not edit!


(when (boundp 'local_sensing_node::DynamicObstacleStateArray)
  (if (not (find-package "LOCAL_SENSING_NODE"))
    (make-package "LOCAL_SENSING_NODE"))
  (shadow 'DynamicObstacleStateArray (find-package "LOCAL_SENSING_NODE")))
(unless (find-package "LOCAL_SENSING_NODE::DYNAMICOBSTACLESTATEARRAY")
  (make-package "LOCAL_SENSING_NODE::DYNAMICOBSTACLESTATEARRAY"))

(in-package "ROS")
;;//! \htmlinclude DynamicObstacleStateArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass local_sensing_node::DynamicObstacleStateArray
  :super ros::object
  :slots (_header _obstacles ))

(defmethod local_sensing_node::DynamicObstacleStateArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:obstacles __obstacles) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _obstacles __obstacles)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:obstacles
   (&rest __obstacles)
   (if (keywordp (car __obstacles))
       (send* _obstacles __obstacles)
     (progn
       (if __obstacles (setq _obstacles (car __obstacles)))
       _obstacles)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; local_sensing_node/DynamicObstacleState[] _obstacles
    (apply #'+ (send-all _obstacles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; local_sensing_node/DynamicObstacleState[] _obstacles
     (write-long (length _obstacles) s)
     (dolist (elem _obstacles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; local_sensing_node/DynamicObstacleState[] _obstacles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles (let (r) (dotimes (i n) (push (instance local_sensing_node::DynamicObstacleState :init) r)) r))
     (dolist (elem- _obstacles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get local_sensing_node::DynamicObstacleStateArray :md5sum-) "2411b68bc8e07baa293691aaa8841ee6")
(setf (get local_sensing_node::DynamicObstacleStateArray :datatype-) "local_sensing_node/DynamicObstacleStateArray")
(setf (get local_sensing_node::DynamicObstacleStateArray :definition-)
      "std_msgs/Header header
local_sensing_node/DynamicObstacleState[] obstacles

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
MSG: local_sensing_node/DynamicObstacleState
int32 obstacle_id
bool is_dynamic
string geometry_type
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 size

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



(provide :local_sensing_node/DynamicObstacleStateArray "2411b68bc8e07baa293691aaa8841ee6")


