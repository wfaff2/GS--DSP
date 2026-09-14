;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::fc_to_oa)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'fc_to_oa (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::FC_TO_OA")
  (make-package "QUADROTOR_MSGS::FC_TO_OA"))

(in-package "ROS")
;;//! \htmlinclude fc_to_oa.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::fc_to_oa
  :super ros::object
  :slots (_header _q0 _q1 _q2 _q3 _gyro_x _gyro_y _gyro_z _acc_x _acc_y _acc_z _down_dist _top_dist _lati _longti _press _vg_x _vg_y _vg_z _aileron _elevator _throttle _rudder _fc_timestamp_ms _vo_flag _roll _pitch _yaw _anti_longti _anti_press ))

(defmethod quadrotor_msgs::fc_to_oa
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:q0 __q0) 0.0)
    ((:q1 __q1) 0.0)
    ((:q2 __q2) 0.0)
    ((:q3 __q3) 0.0)
    ((:gyro_x __gyro_x) 0.0)
    ((:gyro_y __gyro_y) 0.0)
    ((:gyro_z __gyro_z) 0.0)
    ((:acc_x __acc_x) 0.0)
    ((:acc_y __acc_y) 0.0)
    ((:acc_z __acc_z) 0.0)
    ((:down_dist __down_dist) 0.0)
    ((:top_dist __top_dist) 0.0)
    ((:lati __lati) 0.0)
    ((:longti __longti) 0.0)
    ((:press __press) 0.0)
    ((:vg_x __vg_x) 0.0)
    ((:vg_y __vg_y) 0.0)
    ((:vg_z __vg_z) 0.0)
    ((:aileron __aileron) 0)
    ((:elevator __elevator) 0)
    ((:throttle __throttle) 0)
    ((:rudder __rudder) 0)
    ((:fc_timestamp_ms __fc_timestamp_ms) 0)
    ((:vo_flag __vo_flag) 0)
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:anti_longti __anti_longti) 0.0)
    ((:anti_press __anti_press) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _q0 (float __q0))
   (setq _q1 (float __q1))
   (setq _q2 (float __q2))
   (setq _q3 (float __q3))
   (setq _gyro_x (float __gyro_x))
   (setq _gyro_y (float __gyro_y))
   (setq _gyro_z (float __gyro_z))
   (setq _acc_x (float __acc_x))
   (setq _acc_y (float __acc_y))
   (setq _acc_z (float __acc_z))
   (setq _down_dist (float __down_dist))
   (setq _top_dist (float __top_dist))
   (setq _lati (float __lati))
   (setq _longti (float __longti))
   (setq _press (float __press))
   (setq _vg_x (float __vg_x))
   (setq _vg_y (float __vg_y))
   (setq _vg_z (float __vg_z))
   (setq _aileron (round __aileron))
   (setq _elevator (round __elevator))
   (setq _throttle (round __throttle))
   (setq _rudder (round __rudder))
   (setq _fc_timestamp_ms (round __fc_timestamp_ms))
   (setq _vo_flag (round __vo_flag))
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _anti_longti (float __anti_longti))
   (setq _anti_press (float __anti_press))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:q0
   (&optional __q0)
   (if __q0 (setq _q0 __q0)) _q0)
  (:q1
   (&optional __q1)
   (if __q1 (setq _q1 __q1)) _q1)
  (:q2
   (&optional __q2)
   (if __q2 (setq _q2 __q2)) _q2)
  (:q3
   (&optional __q3)
   (if __q3 (setq _q3 __q3)) _q3)
  (:gyro_x
   (&optional __gyro_x)
   (if __gyro_x (setq _gyro_x __gyro_x)) _gyro_x)
  (:gyro_y
   (&optional __gyro_y)
   (if __gyro_y (setq _gyro_y __gyro_y)) _gyro_y)
  (:gyro_z
   (&optional __gyro_z)
   (if __gyro_z (setq _gyro_z __gyro_z)) _gyro_z)
  (:acc_x
   (&optional __acc_x)
   (if __acc_x (setq _acc_x __acc_x)) _acc_x)
  (:acc_y
   (&optional __acc_y)
   (if __acc_y (setq _acc_y __acc_y)) _acc_y)
  (:acc_z
   (&optional __acc_z)
   (if __acc_z (setq _acc_z __acc_z)) _acc_z)
  (:down_dist
   (&optional __down_dist)
   (if __down_dist (setq _down_dist __down_dist)) _down_dist)
  (:top_dist
   (&optional __top_dist)
   (if __top_dist (setq _top_dist __top_dist)) _top_dist)
  (:lati
   (&optional __lati)
   (if __lati (setq _lati __lati)) _lati)
  (:longti
   (&optional __longti)
   (if __longti (setq _longti __longti)) _longti)
  (:press
   (&optional __press)
   (if __press (setq _press __press)) _press)
  (:vg_x
   (&optional __vg_x)
   (if __vg_x (setq _vg_x __vg_x)) _vg_x)
  (:vg_y
   (&optional __vg_y)
   (if __vg_y (setq _vg_y __vg_y)) _vg_y)
  (:vg_z
   (&optional __vg_z)
   (if __vg_z (setq _vg_z __vg_z)) _vg_z)
  (:aileron
   (&optional __aileron)
   (if __aileron (setq _aileron __aileron)) _aileron)
  (:elevator
   (&optional __elevator)
   (if __elevator (setq _elevator __elevator)) _elevator)
  (:throttle
   (&optional __throttle)
   (if __throttle (setq _throttle __throttle)) _throttle)
  (:rudder
   (&optional __rudder)
   (if __rudder (setq _rudder __rudder)) _rudder)
  (:fc_timestamp_ms
   (&optional __fc_timestamp_ms)
   (if __fc_timestamp_ms (setq _fc_timestamp_ms __fc_timestamp_ms)) _fc_timestamp_ms)
  (:vo_flag
   (&optional __vo_flag)
   (if __vo_flag (setq _vo_flag __vo_flag)) _vo_flag)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:anti_longti
   (&optional __anti_longti)
   (if __anti_longti (setq _anti_longti __anti_longti)) _anti_longti)
  (:anti_press
   (&optional __anti_press)
   (if __anti_press (setq _anti_press __anti_press)) _anti_press)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _q0
    4
    ;; float32 _q1
    4
    ;; float32 _q2
    4
    ;; float32 _q3
    4
    ;; float32 _gyro_x
    4
    ;; float32 _gyro_y
    4
    ;; float32 _gyro_z
    4
    ;; float32 _acc_x
    4
    ;; float32 _acc_y
    4
    ;; float32 _acc_z
    4
    ;; float32 _down_dist
    4
    ;; float32 _top_dist
    4
    ;; float32 _lati
    4
    ;; float32 _longti
    4
    ;; float32 _press
    4
    ;; float32 _vg_x
    4
    ;; float32 _vg_y
    4
    ;; float32 _vg_z
    4
    ;; int16 _aileron
    2
    ;; int16 _elevator
    2
    ;; int16 _throttle
    2
    ;; int16 _rudder
    2
    ;; uint32 _fc_timestamp_ms
    4
    ;; int16 _vo_flag
    2
    ;; float32 _roll
    4
    ;; float32 _pitch
    4
    ;; float32 _yaw
    4
    ;; float32 _anti_longti
    4
    ;; float32 _anti_press
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _q0
       (sys::poke _q0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _q1
       (sys::poke _q1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _q2
       (sys::poke _q2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _q3
       (sys::poke _q3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gyro_x
       (sys::poke _gyro_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gyro_y
       (sys::poke _gyro_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gyro_z
       (sys::poke _gyro_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acc_x
       (sys::poke _acc_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acc_y
       (sys::poke _acc_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acc_z
       (sys::poke _acc_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _down_dist
       (sys::poke _down_dist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _top_dist
       (sys::poke _top_dist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lati
       (sys::poke _lati (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _longti
       (sys::poke _longti (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _press
       (sys::poke _press (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vg_x
       (sys::poke _vg_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vg_y
       (sys::poke _vg_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vg_z
       (sys::poke _vg_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int16 _aileron
       (write-word _aileron s)
     ;; int16 _elevator
       (write-word _elevator s)
     ;; int16 _throttle
       (write-word _throttle s)
     ;; int16 _rudder
       (write-word _rudder s)
     ;; uint32 _fc_timestamp_ms
       (write-long _fc_timestamp_ms s)
     ;; int16 _vo_flag
       (write-word _vo_flag s)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _anti_longti
       (sys::poke _anti_longti (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _anti_press
       (sys::poke _anti_press (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _q0
     (setq _q0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _q1
     (setq _q1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _q2
     (setq _q2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _q3
     (setq _q3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gyro_x
     (setq _gyro_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gyro_y
     (setq _gyro_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gyro_z
     (setq _gyro_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acc_x
     (setq _acc_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acc_y
     (setq _acc_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acc_z
     (setq _acc_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _down_dist
     (setq _down_dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _top_dist
     (setq _top_dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lati
     (setq _lati (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _longti
     (setq _longti (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _press
     (setq _press (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vg_x
     (setq _vg_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vg_y
     (setq _vg_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vg_z
     (setq _vg_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int16 _aileron
     (setq _aileron (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _elevator
     (setq _elevator (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _throttle
     (setq _throttle (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _rudder
     (setq _rudder (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint32 _fc_timestamp_ms
     (setq _fc_timestamp_ms (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16 _vo_flag
     (setq _vo_flag (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _anti_longti
     (setq _anti_longti (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _anti_press
     (setq _anti_press (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get quadrotor_msgs::fc_to_oa :md5sum-) "f2d437dc8b62cfe7d33ce34dfc5bd422")
(setf (get quadrotor_msgs::fc_to_oa :datatype-) "quadrotor_msgs/fc_to_oa")
(setf (get quadrotor_msgs::fc_to_oa :definition-)
      "std_msgs/Header header
float32 q0
float32 q1
float32 q2
float32 q3
float32 gyro_x
float32 gyro_y
float32 gyro_z
float32 acc_x
float32 acc_y
float32 acc_z
float32 down_dist
float32 top_dist
float32 lati
float32 longti
float32 press
float32 vg_x
float32 vg_y
float32 vg_z
int16 aileron
int16 elevator
int16 throttle
int16 rudder
uint32 fc_timestamp_ms
int16 vo_flag
float32 roll
float32 pitch
float32 yaw
float32 anti_longti
float32 anti_press

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



(provide :quadrotor_msgs/fc_to_oa "f2d437dc8b62cfe7d33ce34dfc5bd422")


