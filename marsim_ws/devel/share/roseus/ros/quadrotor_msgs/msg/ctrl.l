;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::ctrl)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'ctrl (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::CTRL")
  (make-package "QUADROTOR_MSGS::CTRL"))

(in-package "ROS")
;;//! \htmlinclude ctrl.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*FLAG_TYPE_ATTI_VERT_POS*" (find-package "QUADROTOR_MSGS::CTRL"))
(shadow '*FLAG_TYPE_ATTI_VERT_POS* (find-package "QUADROTOR_MSGS::CTRL"))
(defconstant quadrotor_msgs::ctrl::*FLAG_TYPE_ATTI_VERT_POS* 18)
(intern "*FLAG_TYPE_VELO_YAW_RATE*" (find-package "QUADROTOR_MSGS::CTRL"))
(shadow '*FLAG_TYPE_VELO_YAW_RATE* (find-package "QUADROTOR_MSGS::CTRL"))
(defconstant quadrotor_msgs::ctrl::*FLAG_TYPE_VELO_YAW_RATE* 72)
(intern "*FLAG_TYPE_HORIZ_ANG_VERT_VEL_YAW_ANG*" (find-package "QUADROTOR_MSGS::CTRL"))
(shadow '*FLAG_TYPE_HORIZ_ANG_VERT_VEL_YAW_ANG* (find-package "QUADROTOR_MSGS::CTRL"))
(defconstant quadrotor_msgs::ctrl::*FLAG_TYPE_HORIZ_ANG_VERT_VEL_YAW_ANG* 1)
(intern "*FLAG_VERTICAL_THRUST_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY*" (find-package "QUADROTOR_MSGS::CTRL"))
(shadow '*FLAG_VERTICAL_THRUST_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY* (find-package "QUADROTOR_MSGS::CTRL"))
(defconstant quadrotor_msgs::ctrl::*FLAG_VERTICAL_THRUST_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY* 42)
(intern "*FLAG_VERTICAL_POSITION_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY*" (find-package "QUADROTOR_MSGS::CTRL"))
(shadow '*FLAG_VERTICAL_POSITION_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY* (find-package "QUADROTOR_MSGS::CTRL"))
(defconstant quadrotor_msgs::ctrl::*FLAG_VERTICAL_POSITION_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY* 26)

(defun quadrotor_msgs::ctrl-to-symbol (const)
  (cond
        ((= const 18) 'quadrotor_msgs::ctrl::*FLAG_TYPE_ATTI_VERT_POS*)
        ((= const 72) 'quadrotor_msgs::ctrl::*FLAG_TYPE_VELO_YAW_RATE*)
        ((= const 1) 'quadrotor_msgs::ctrl::*FLAG_TYPE_HORIZ_ANG_VERT_VEL_YAW_ANG*)
        ((= const 42) 'quadrotor_msgs::ctrl::*FLAG_VERTICAL_THRUST_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY*)
        ((= const 26) 'quadrotor_msgs::ctrl::*FLAG_VERTICAL_POSITION_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY*)
        (t nil)))

(defclass quadrotor_msgs::ctrl
  :super ros::object
  :slots (_header _x_axis_ctrl _y_axis_ctrl _z_axis_ctrl _vertical_ctrl _x_feed_forward _y_feed_forward _flag ))

(defmethod quadrotor_msgs::ctrl
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:x_axis_ctrl __x_axis_ctrl) 0.0)
    ((:y_axis_ctrl __y_axis_ctrl) 0.0)
    ((:z_axis_ctrl __z_axis_ctrl) 0.0)
    ((:vertical_ctrl __vertical_ctrl) 0.0)
    ((:x_feed_forward __x_feed_forward) 0.0)
    ((:y_feed_forward __y_feed_forward) 0.0)
    ((:flag __flag) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _x_axis_ctrl (float __x_axis_ctrl))
   (setq _y_axis_ctrl (float __y_axis_ctrl))
   (setq _z_axis_ctrl (float __z_axis_ctrl))
   (setq _vertical_ctrl (float __vertical_ctrl))
   (setq _x_feed_forward (float __x_feed_forward))
   (setq _y_feed_forward (float __y_feed_forward))
   (setq _flag (round __flag))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:x_axis_ctrl
   (&optional __x_axis_ctrl)
   (if __x_axis_ctrl (setq _x_axis_ctrl __x_axis_ctrl)) _x_axis_ctrl)
  (:y_axis_ctrl
   (&optional __y_axis_ctrl)
   (if __y_axis_ctrl (setq _y_axis_ctrl __y_axis_ctrl)) _y_axis_ctrl)
  (:z_axis_ctrl
   (&optional __z_axis_ctrl)
   (if __z_axis_ctrl (setq _z_axis_ctrl __z_axis_ctrl)) _z_axis_ctrl)
  (:vertical_ctrl
   (&optional __vertical_ctrl)
   (if __vertical_ctrl (setq _vertical_ctrl __vertical_ctrl)) _vertical_ctrl)
  (:x_feed_forward
   (&optional __x_feed_forward)
   (if __x_feed_forward (setq _x_feed_forward __x_feed_forward)) _x_feed_forward)
  (:y_feed_forward
   (&optional __y_feed_forward)
   (if __y_feed_forward (setq _y_feed_forward __y_feed_forward)) _y_feed_forward)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _x_axis_ctrl
    4
    ;; float32 _y_axis_ctrl
    4
    ;; float32 _z_axis_ctrl
    4
    ;; float32 _vertical_ctrl
    4
    ;; float32 _x_feed_forward
    4
    ;; float32 _y_feed_forward
    4
    ;; uint8 _flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _x_axis_ctrl
       (sys::poke _x_axis_ctrl (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_axis_ctrl
       (sys::poke _y_axis_ctrl (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_axis_ctrl
       (sys::poke _z_axis_ctrl (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vertical_ctrl
       (sys::poke _vertical_ctrl (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_feed_forward
       (sys::poke _x_feed_forward (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_feed_forward
       (sys::poke _y_feed_forward (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _flag
       (write-byte _flag s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _x_axis_ctrl
     (setq _x_axis_ctrl (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_axis_ctrl
     (setq _y_axis_ctrl (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_axis_ctrl
     (setq _z_axis_ctrl (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vertical_ctrl
     (setq _vertical_ctrl (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_feed_forward
     (setq _x_feed_forward (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_feed_forward
     (setq _y_feed_forward (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _flag
     (setq _flag (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get quadrotor_msgs::ctrl :md5sum-) "0f530cfce16387cbbfb41657d4d60cbc")
(setf (get quadrotor_msgs::ctrl :datatype-) "quadrotor_msgs/ctrl")
(setf (get quadrotor_msgs::ctrl :definition-)
      "std_msgs/Header header
float32 x_axis_ctrl
float32 y_axis_ctrl
float32 z_axis_ctrl
float32 vertical_ctrl
float32 x_feed_forward
float32 y_feed_forward
uint8   flag
uint8 FLAG_TYPE_ATTI_VERT_POS = 18   # atti control
uint8 FLAG_TYPE_VELO_YAW_RATE = 72   # velocity control
uint8 FLAG_TYPE_HORIZ_ANG_VERT_VEL_YAW_ANG = 1
uint8 FLAG_VERTICAL_THRUST_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY = 42
uint8 FLAG_VERTICAL_POSITION_HORIZONTAL_ANGLE_YAW_RATE_HORIZONTAL_BODY = 26

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



(provide :quadrotor_msgs/ctrl "0f530cfce16387cbbfb41657d4d60cbc")


