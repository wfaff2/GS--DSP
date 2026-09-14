;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::aec)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'aec (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::AEC")
  (make-package "QUADROTOR_MSGS::AEC"))

(in-package "ROS")
;;//! \htmlinclude aec.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::aec
  :super ros::object
  :slots (_header _sensor_pos _shutter_val _dgain_val _dgain_a _again_val _again_a ))

(defmethod quadrotor_msgs::aec
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:sensor_pos __sensor_pos) 0)
    ((:shutter_val __shutter_val) 0)
    ((:dgain_val __dgain_val) 0)
    ((:dgain_a __dgain_a) 0)
    ((:again_val __again_val) 0)
    ((:again_a __again_a) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _sensor_pos (round __sensor_pos))
   (setq _shutter_val (round __shutter_val))
   (setq _dgain_val (round __dgain_val))
   (setq _dgain_a (round __dgain_a))
   (setq _again_val (round __again_val))
   (setq _again_a (round __again_a))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:sensor_pos
   (&optional __sensor_pos)
   (if __sensor_pos (setq _sensor_pos __sensor_pos)) _sensor_pos)
  (:shutter_val
   (&optional __shutter_val)
   (if __shutter_val (setq _shutter_val __shutter_val)) _shutter_val)
  (:dgain_val
   (&optional __dgain_val)
   (if __dgain_val (setq _dgain_val __dgain_val)) _dgain_val)
  (:dgain_a
   (&optional __dgain_a)
   (if __dgain_a (setq _dgain_a __dgain_a)) _dgain_a)
  (:again_val
   (&optional __again_val)
   (if __again_val (setq _again_val __again_val)) _again_val)
  (:again_a
   (&optional __again_a)
   (if __again_a (setq _again_a __again_a)) _again_a)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _sensor_pos
    2
    ;; int16 _shutter_val
    2
    ;; int16 _dgain_val
    2
    ;; int16 _dgain_a
    2
    ;; int16 _again_val
    2
    ;; int16 _again_a
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _sensor_pos
       (write-word _sensor_pos s)
     ;; int16 _shutter_val
       (write-word _shutter_val s)
     ;; int16 _dgain_val
       (write-word _dgain_val s)
     ;; int16 _dgain_a
       (write-word _dgain_a s)
     ;; int16 _again_val
       (write-word _again_val s)
     ;; int16 _again_a
       (write-word _again_a s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _sensor_pos
     (setq _sensor_pos (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _shutter_val
     (setq _shutter_val (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _dgain_val
     (setq _dgain_val (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _dgain_a
     (setq _dgain_a (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _again_val
     (setq _again_val (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _again_a
     (setq _again_a (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get quadrotor_msgs::aec :md5sum-) "bb5f8b724b61c9413d80e9b092077813")
(setf (get quadrotor_msgs::aec :datatype-) "quadrotor_msgs/aec")
(setf (get quadrotor_msgs::aec :definition-)
      "std_msgs/Header header
int16 sensor_pos
int16 shutter_val
int16 dgain_val
int16 dgain_a
int16 again_val
int16 again_a

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



(provide :quadrotor_msgs/aec "bb5f8b724b61c9413d80e9b092077813")


