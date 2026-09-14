;; Auto-generated. Do not edit!


(when (boundp 'onboard_detector::TrackedObstacle)
  (if (not (find-package "ONBOARD_DETECTOR"))
    (make-package "ONBOARD_DETECTOR"))
  (shadow 'TrackedObstacle (find-package "ONBOARD_DETECTOR")))
(unless (find-package "ONBOARD_DETECTOR::TRACKEDOBSTACLE")
  (make-package "ONBOARD_DETECTOR::TRACKEDOBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude TrackedObstacle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass onboard_detector::TrackedObstacle
  :super ros::object
  :slots (_slot_id _is_dynamic _position _velocity _size ))

(defmethod onboard_detector::TrackedObstacle
  (:init
   (&key
    ((:slot_id __slot_id) 0)
    ((:is_dynamic __is_dynamic) nil)
    ((:position __position) (instance geometry_msgs::Vector3 :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:size __size) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _slot_id (round __slot_id))
   (setq _is_dynamic __is_dynamic)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _size __size)
   self)
  (:slot_id
   (&optional __slot_id)
   (if __slot_id (setq _slot_id __slot_id)) _slot_id)
  (:is_dynamic
   (&optional (__is_dynamic :null))
   (if (not (eq __is_dynamic :null)) (setq _is_dynamic __is_dynamic)) _is_dynamic)
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
  (:size
   (&rest __size)
   (if (keywordp (car __size))
       (send* _size __size)
     (progn
       (if __size (setq _size (car __size)))
       _size)))
  (:serialization-length
   ()
   (+
    ;; int32 _slot_id
    4
    ;; bool _is_dynamic
    1
    ;; geometry_msgs/Vector3 _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; geometry_msgs/Vector3 _size
    (send _size :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _slot_id
       (write-long _slot_id s)
     ;; bool _is_dynamic
       (if _is_dynamic (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Vector3 _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; geometry_msgs/Vector3 _size
       (send _size :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _slot_id
     (setq _slot_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _is_dynamic
     (setq _is_dynamic (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Vector3 _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; geometry_msgs/Vector3 _size
     (send _size :deserialize buf ptr-) (incf ptr- (send _size :serialization-length))
   ;;
   self)
  )

(setf (get onboard_detector::TrackedObstacle :md5sum-) "50d60af2ec9911de5b888d1721cba72d")
(setf (get onboard_detector::TrackedObstacle :datatype-) "onboard_detector/TrackedObstacle")
(setf (get onboard_detector::TrackedObstacle :definition-)
      "int32 slot_id
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



(provide :onboard_detector/TrackedObstacle "50d60af2ec9911de5b888d1721cba72d")


