;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::oa_result)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'oa_result (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::OA_RESULT")
  (make-package "QUADROTOR_MSGS::OA_RESULT"))

(in-package "ROS")
;;//! \htmlinclude oa_result.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::oa_result
  :super ros::object
  :slots (_header _direction _reserved _tof_dist _image_index _image_timestamp _object_distance _stop _failed_distance _failed_distance_app _speed_limit _alarm _object_distance_app _high_atti _is_release_brake ))

(defmethod quadrotor_msgs::oa_result
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:direction __direction) 0)
    ((:reserved __reserved) 0)
    ((:tof_dist __tof_dist) 0.0)
    ((:image_index __image_index) 0)
    ((:image_timestamp __image_timestamp) 0)
    ((:object_distance __object_distance) (make-array 3 :initial-element 0 :element-type :integer))
    ((:stop __stop) 0)
    ((:failed_distance __failed_distance) 0)
    ((:failed_distance_app __failed_distance_app) 0)
    ((:speed_limit __speed_limit) 0)
    ((:alarm __alarm) 0)
    ((:object_distance_app __object_distance_app) (make-array 4 :initial-element 0 :element-type :integer))
    ((:high_atti __high_atti) 0)
    ((:is_release_brake __is_release_brake) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _direction (round __direction))
   (setq _reserved (round __reserved))
   (setq _tof_dist (float __tof_dist))
   (setq _image_index (round __image_index))
   (setq _image_timestamp (round __image_timestamp))
   (setq _object_distance __object_distance)
   (setq _stop (round __stop))
   (setq _failed_distance (round __failed_distance))
   (setq _failed_distance_app (round __failed_distance_app))
   (setq _speed_limit (round __speed_limit))
   (setq _alarm (round __alarm))
   (setq _object_distance_app __object_distance_app)
   (setq _high_atti (round __high_atti))
   (setq _is_release_brake (round __is_release_brake))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:reserved
   (&optional __reserved)
   (if __reserved (setq _reserved __reserved)) _reserved)
  (:tof_dist
   (&optional __tof_dist)
   (if __tof_dist (setq _tof_dist __tof_dist)) _tof_dist)
  (:image_index
   (&optional __image_index)
   (if __image_index (setq _image_index __image_index)) _image_index)
  (:image_timestamp
   (&optional __image_timestamp)
   (if __image_timestamp (setq _image_timestamp __image_timestamp)) _image_timestamp)
  (:object_distance
   (&optional __object_distance)
   (if __object_distance (setq _object_distance __object_distance)) _object_distance)
  (:stop
   (&optional __stop)
   (if __stop (setq _stop __stop)) _stop)
  (:failed_distance
   (&optional __failed_distance)
   (if __failed_distance (setq _failed_distance __failed_distance)) _failed_distance)
  (:failed_distance_app
   (&optional __failed_distance_app)
   (if __failed_distance_app (setq _failed_distance_app __failed_distance_app)) _failed_distance_app)
  (:speed_limit
   (&optional __speed_limit)
   (if __speed_limit (setq _speed_limit __speed_limit)) _speed_limit)
  (:alarm
   (&optional __alarm)
   (if __alarm (setq _alarm __alarm)) _alarm)
  (:object_distance_app
   (&optional __object_distance_app)
   (if __object_distance_app (setq _object_distance_app __object_distance_app)) _object_distance_app)
  (:high_atti
   (&optional __high_atti)
   (if __high_atti (setq _high_atti __high_atti)) _high_atti)
  (:is_release_brake
   (&optional __is_release_brake)
   (if __is_release_brake (setq _is_release_brake __is_release_brake)) _is_release_brake)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _direction
    4
    ;; int32 _reserved
    4
    ;; float32 _tof_dist
    4
    ;; int32 _image_index
    4
    ;; uint32 _image_timestamp
    4
    ;; int16[3] _object_distance
    (* 2    3)
    ;; uint8 _stop
    1
    ;; uint8 _failed_distance
    1
    ;; uint8 _failed_distance_app
    1
    ;; uint8 _speed_limit
    1
    ;; uint8 _alarm
    1
    ;; int16[4] _object_distance_app
    (* 2    4)
    ;; int32 _high_atti
    4
    ;; int32 _is_release_brake
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _direction
       (write-long _direction s)
     ;; int32 _reserved
       (write-long _reserved s)
     ;; float32 _tof_dist
       (sys::poke _tof_dist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _image_index
       (write-long _image_index s)
     ;; uint32 _image_timestamp
       (write-long _image_timestamp s)
     ;; int16[3] _object_distance
     (dotimes (i 3)
       (write-word (elt _object_distance i) s)
       )
     ;; uint8 _stop
       (write-byte _stop s)
     ;; uint8 _failed_distance
       (write-byte _failed_distance s)
     ;; uint8 _failed_distance_app
       (write-byte _failed_distance_app s)
     ;; uint8 _speed_limit
       (write-byte _speed_limit s)
     ;; uint8 _alarm
       (write-byte _alarm s)
     ;; int16[4] _object_distance_app
     (dotimes (i 4)
       (write-word (elt _object_distance_app i) s)
       )
     ;; int32 _high_atti
       (write-long _high_atti s)
     ;; int32 _is_release_brake
       (write-long _is_release_brake s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _direction
     (setq _direction (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _reserved
     (setq _reserved (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _tof_dist
     (setq _tof_dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _image_index
     (setq _image_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _image_timestamp
     (setq _image_timestamp (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16[3] _object_distance
   (dotimes (i (length _object_distance))
     (setf (elt _object_distance i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint8 _stop
     (setq _stop (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _failed_distance
     (setq _failed_distance (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _failed_distance_app
     (setq _failed_distance_app (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _speed_limit
     (setq _speed_limit (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _alarm
     (setq _alarm (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int16[4] _object_distance_app
   (dotimes (i (length _object_distance_app))
     (setf (elt _object_distance_app i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; int32 _high_atti
     (setq _high_atti (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _is_release_brake
     (setq _is_release_brake (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get quadrotor_msgs::oa_result :md5sum-) "1f8843ccbd246ed329e5be927b5cd5cb")
(setf (get quadrotor_msgs::oa_result :datatype-) "quadrotor_msgs/oa_result")
(setf (get quadrotor_msgs::oa_result :definition-)
      "std_msgs/Header header
uint32 direction
int32 reserved
float32 tof_dist
int32 image_index
uint32 image_timestamp
int16[3] object_distance        # uint8 NUMBER_SUB_DIRECTIONS = 3, for FC
uint8 stop
uint8 failed_distance
uint8 failed_distance_app
uint8 speed_limit
uint8 alarm
int16[4] object_distance_app    # uint8 NUMBER_SUB_DIRECTIONS_FOR_APP = 4, for APP
int32 high_atti
int32 is_release_brake
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



(provide :quadrotor_msgs/oa_result "1f8843ccbd246ed329e5be927b5cd5cb")


