;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::vio_result)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'vio_result (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::VIO_RESULT")
  (make-package "QUADROTOR_MSGS::VIO_RESULT"))

(in-package "ROS")
;;//! \htmlinclude vio_result.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::vio_result
  :super ros::object
  :slots (_header _pose_id _time_stamp_ms _flag _p_wi _v_wi _q_wi _q_ic _p_ic _vps_from_vio ))

(defmethod quadrotor_msgs::vio_result
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose_id __pose_id) 0)
    ((:time_stamp_ms __time_stamp_ms) 0)
    ((:flag __flag) 0)
    ((:p_wi __p_wi) (instance geometry_msgs::Vector3 :init))
    ((:v_wi __v_wi) (instance geometry_msgs::Vector3 :init))
    ((:q_wi __q_wi) (instance geometry_msgs::Quaternion :init))
    ((:q_ic __q_ic) (let (r) (dotimes (i 6) (push (instance geometry_msgs::Quaternion :init) r)) r))
    ((:p_ic __p_ic) (let (r) (dotimes (i 6) (push (instance geometry_msgs::Vector3 :init) r)) r))
    ((:vps_from_vio __vps_from_vio) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose_id (round __pose_id))
   (setq _time_stamp_ms (round __time_stamp_ms))
   (setq _flag (round __flag))
   (setq _p_wi __p_wi)
   (setq _v_wi __v_wi)
   (setq _q_wi __q_wi)
   (setq _q_ic __q_ic)
   (setq _p_ic __p_ic)
   (setq _vps_from_vio (float __vps_from_vio))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose_id
   (&optional __pose_id)
   (if __pose_id (setq _pose_id __pose_id)) _pose_id)
  (:time_stamp_ms
   (&optional __time_stamp_ms)
   (if __time_stamp_ms (setq _time_stamp_ms __time_stamp_ms)) _time_stamp_ms)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:p_wi
   (&rest __p_wi)
   (if (keywordp (car __p_wi))
       (send* _p_wi __p_wi)
     (progn
       (if __p_wi (setq _p_wi (car __p_wi)))
       _p_wi)))
  (:v_wi
   (&rest __v_wi)
   (if (keywordp (car __v_wi))
       (send* _v_wi __v_wi)
     (progn
       (if __v_wi (setq _v_wi (car __v_wi)))
       _v_wi)))
  (:q_wi
   (&rest __q_wi)
   (if (keywordp (car __q_wi))
       (send* _q_wi __q_wi)
     (progn
       (if __q_wi (setq _q_wi (car __q_wi)))
       _q_wi)))
  (:q_ic
   (&rest __q_ic)
   (if (keywordp (car __q_ic))
       (send* _q_ic __q_ic)
     (progn
       (if __q_ic (setq _q_ic (car __q_ic)))
       _q_ic)))
  (:p_ic
   (&rest __p_ic)
   (if (keywordp (car __p_ic))
       (send* _p_ic __p_ic)
     (progn
       (if __p_ic (setq _p_ic (car __p_ic)))
       _p_ic)))
  (:vps_from_vio
   (&optional __vps_from_vio)
   (if __vps_from_vio (setq _vps_from_vio __vps_from_vio)) _vps_from_vio)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _pose_id
    4
    ;; uint32 _time_stamp_ms
    4
    ;; uint8 _flag
    1
    ;; geometry_msgs/Vector3 _p_wi
    (send _p_wi :serialization-length)
    ;; geometry_msgs/Vector3 _v_wi
    (send _v_wi :serialization-length)
    ;; geometry_msgs/Quaternion _q_wi
    (send _q_wi :serialization-length)
    ;; geometry_msgs/Quaternion[6] _q_ic
    (apply #'+ (send-all _q_ic :serialization-length))
    ;; geometry_msgs/Vector3[6] _p_ic
    (apply #'+ (send-all _p_ic :serialization-length))
    ;; float32 _vps_from_vio
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _pose_id
       (write-long _pose_id s)
     ;; uint32 _time_stamp_ms
       (write-long _time_stamp_ms s)
     ;; uint8 _flag
       (write-byte _flag s)
     ;; geometry_msgs/Vector3 _p_wi
       (send _p_wi :serialize s)
     ;; geometry_msgs/Vector3 _v_wi
       (send _v_wi :serialize s)
     ;; geometry_msgs/Quaternion _q_wi
       (send _q_wi :serialize s)
     ;; geometry_msgs/Quaternion[6] _q_ic
     (dolist (elem _q_ic)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3[6] _p_ic
     (dolist (elem _p_ic)
       (send elem :serialize s)
       )
     ;; float32 _vps_from_vio
       (sys::poke _vps_from_vio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _pose_id
     (setq _pose_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _time_stamp_ms
     (setq _time_stamp_ms (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _flag
     (setq _flag (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Vector3 _p_wi
     (send _p_wi :deserialize buf ptr-) (incf ptr- (send _p_wi :serialization-length))
   ;; geometry_msgs/Vector3 _v_wi
     (send _v_wi :deserialize buf ptr-) (incf ptr- (send _v_wi :serialization-length))
   ;; geometry_msgs/Quaternion _q_wi
     (send _q_wi :deserialize buf ptr-) (incf ptr- (send _q_wi :serialization-length))
   ;; geometry_msgs/Quaternion[6] _q_ic
   (dotimes (i 6)
     (send (elt _q_ic i) :deserialize buf ptr-) (incf ptr- (send (elt _q_ic i) :serialization-length))
     )
   ;; geometry_msgs/Vector3[6] _p_ic
   (dotimes (i 6)
     (send (elt _p_ic i) :deserialize buf ptr-) (incf ptr- (send (elt _p_ic i) :serialization-length))
     )
   ;; float32 _vps_from_vio
     (setq _vps_from_vio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get quadrotor_msgs::vio_result :md5sum-) "324b3402a89081d3fe2b8181d4ced579")
(setf (get quadrotor_msgs::vio_result :datatype-) "quadrotor_msgs/vio_result")
(setf (get quadrotor_msgs::vio_result :definition-)
      "std_msgs/Header header
uint32 pose_id
uint32 time_stamp_ms
uint8 flag
geometry_msgs/Vector3 p_wi
geometry_msgs/Vector3 v_wi
geometry_msgs/Quaternion q_wi
geometry_msgs/Quaternion[6] q_ic
geometry_msgs/Vector3[6] p_ic
float32 vps_from_vio

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
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :quadrotor_msgs/vio_result "324b3402a89081d3fe2b8181d4ced579")


