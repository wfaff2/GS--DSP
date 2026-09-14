;; Auto-generated. Do not edit!


(when (boundp 'onboard_detector::TrackedObstacleArray)
  (if (not (find-package "ONBOARD_DETECTOR"))
    (make-package "ONBOARD_DETECTOR"))
  (shadow 'TrackedObstacleArray (find-package "ONBOARD_DETECTOR")))
(unless (find-package "ONBOARD_DETECTOR::TRACKEDOBSTACLEARRAY")
  (make-package "ONBOARD_DETECTOR::TRACKEDOBSTACLEARRAY"))

(in-package "ROS")
;;//! \htmlinclude TrackedObstacleArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass onboard_detector::TrackedObstacleArray
  :super ros::object
  :slots (_header _obstacles ))

(defmethod onboard_detector::TrackedObstacleArray
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
    ;; onboard_detector/TrackedObstacle[] _obstacles
    (apply #'+ (send-all _obstacles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; onboard_detector/TrackedObstacle[] _obstacles
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
   ;; onboard_detector/TrackedObstacle[] _obstacles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles (let (r) (dotimes (i n) (push (instance onboard_detector::TrackedObstacle :init) r)) r))
     (dolist (elem- _obstacles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get onboard_detector::TrackedObstacleArray :md5sum-) "f6a3a6c3418c99b8e13b32d276fbf0c1")
(setf (get onboard_detector::TrackedObstacleArray :datatype-) "onboard_detector/TrackedObstacleArray")
(setf (get onboard_detector::TrackedObstacleArray :definition-)
      "std_msgs/Header header
onboard_detector/TrackedObstacle[] obstacles

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
MSG: onboard_detector/TrackedObstacle
int32 slot_id
bool is_dynamic
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 size

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



(provide :onboard_detector/TrackedObstacleArray "f6a3a6c3418c99b8e13b32d276fbf0c1")


