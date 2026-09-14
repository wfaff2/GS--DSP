;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::QuadrotorState)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'QuadrotorState (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::QUADROTORSTATE")
  (make-package "QUADROTOR_MSGS::QUADROTORSTATE"))

(in-package "ROS")
;;//! \htmlinclude QuadrotorState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass quadrotor_msgs::QuadrotorState
  :super ros::object
  :slots (_thrust _velocity_norm _acceleration_norm _jerk_norm _position _velocity _acceleration _jerk _snap _attitude _angular_velocity ))

(defmethod quadrotor_msgs::QuadrotorState
  (:init
   (&key
    ((:thrust __thrust) 0.0)
    ((:velocity_norm __velocity_norm) 0.0)
    ((:acceleration_norm __acceleration_norm) 0.0)
    ((:jerk_norm __jerk_norm) 0.0)
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:acceleration __acceleration) (instance geometry_msgs::Vector3 :init))
    ((:jerk __jerk) (instance geometry_msgs::Vector3 :init))
    ((:snap __snap) (instance geometry_msgs::Vector3 :init))
    ((:attitude __attitude) (instance geometry_msgs::Vector3 :init))
    ((:angular_velocity __angular_velocity) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _thrust (float __thrust))
   (setq _velocity_norm (float __velocity_norm))
   (setq _acceleration_norm (float __acceleration_norm))
   (setq _jerk_norm (float __jerk_norm))
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _acceleration __acceleration)
   (setq _jerk __jerk)
   (setq _snap __snap)
   (setq _attitude __attitude)
   (setq _angular_velocity __angular_velocity)
   self)
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
  (:velocity_norm
   (&optional __velocity_norm)
   (if __velocity_norm (setq _velocity_norm __velocity_norm)) _velocity_norm)
  (:acceleration_norm
   (&optional __acceleration_norm)
   (if __acceleration_norm (setq _acceleration_norm __acceleration_norm)) _acceleration_norm)
  (:jerk_norm
   (&optional __jerk_norm)
   (if __jerk_norm (setq _jerk_norm __jerk_norm)) _jerk_norm)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:acceleration
   (&rest __acceleration)
   (if (keywordp (car __acceleration))
       (send* _acceleration __acceleration)
     (progn
       (if __acceleration (setq _acceleration (car __acceleration)))
       _acceleration)))
  (:jerk
   (&rest __jerk)
   (if (keywordp (car __jerk))
       (send* _jerk __jerk)
     (progn
       (if __jerk (setq _jerk (car __jerk)))
       _jerk)))
  (:snap
   (&rest __snap)
   (if (keywordp (car __snap))
       (send* _snap __snap)
     (progn
       (if __snap (setq _snap (car __snap)))
       _snap)))
  (:attitude
   (&rest __attitude)
   (if (keywordp (car __attitude))
       (send* _attitude __attitude)
     (progn
       (if __attitude (setq _attitude (car __attitude)))
       _attitude)))
  (:angular_velocity
   (&rest __angular_velocity)
   (if (keywordp (car __angular_velocity))
       (send* _angular_velocity __angular_velocity)
     (progn
       (if __angular_velocity (setq _angular_velocity (car __angular_velocity)))
       _angular_velocity)))
  (:serialization-length
   ()
   (+
    ;; float64 _thrust
    8
    ;; float64 _velocity_norm
    8
    ;; float64 _acceleration_norm
    8
    ;; float64 _jerk_norm
    8
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; geometry_msgs/Vector3 _acceleration
    (send _acceleration :serialization-length)
    ;; geometry_msgs/Vector3 _jerk
    (send _jerk :serialization-length)
    ;; geometry_msgs/Vector3 _snap
    (send _snap :serialization-length)
    ;; geometry_msgs/Vector3 _attitude
    (send _attitude :serialization-length)
    ;; geometry_msgs/Vector3 _angular_velocity
    (send _angular_velocity :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _thrust
       (sys::poke _thrust (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _velocity_norm
       (sys::poke _velocity_norm (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _acceleration_norm
       (sys::poke _acceleration_norm (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _jerk_norm
       (sys::poke _jerk_norm (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; geometry_msgs/Vector3 _acceleration
       (send _acceleration :serialize s)
     ;; geometry_msgs/Vector3 _jerk
       (send _jerk :serialize s)
     ;; geometry_msgs/Vector3 _snap
       (send _snap :serialize s)
     ;; geometry_msgs/Vector3 _attitude
       (send _attitude :serialize s)
     ;; geometry_msgs/Vector3 _angular_velocity
       (send _angular_velocity :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _thrust
     (setq _thrust (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _velocity_norm
     (setq _velocity_norm (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _acceleration_norm
     (setq _acceleration_norm (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _jerk_norm
     (setq _jerk_norm (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; geometry_msgs/Vector3 _acceleration
     (send _acceleration :deserialize buf ptr-) (incf ptr- (send _acceleration :serialization-length))
   ;; geometry_msgs/Vector3 _jerk
     (send _jerk :deserialize buf ptr-) (incf ptr- (send _jerk :serialization-length))
   ;; geometry_msgs/Vector3 _snap
     (send _snap :deserialize buf ptr-) (incf ptr- (send _snap :serialization-length))
   ;; geometry_msgs/Vector3 _attitude
     (send _attitude :deserialize buf ptr-) (incf ptr- (send _attitude :serialization-length))
   ;; geometry_msgs/Vector3 _angular_velocity
     (send _angular_velocity :deserialize buf ptr-) (incf ptr- (send _angular_velocity :serialization-length))
   ;;
   self)
  )

(setf (get quadrotor_msgs::QuadrotorState :md5sum-) "9f6239376b083d84c39413d50a0b4367")
(setf (get quadrotor_msgs::QuadrotorState :datatype-) "quadrotor_msgs/QuadrotorState")
(setf (get quadrotor_msgs::QuadrotorState :definition-)
      "float64 thrust
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



(provide :quadrotor_msgs/QuadrotorState "9f6239376b083d84c39413d50a0b4367")


