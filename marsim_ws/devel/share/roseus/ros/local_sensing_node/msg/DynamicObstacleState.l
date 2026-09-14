;; Auto-generated. Do not edit!


(when (boundp 'local_sensing_node::DynamicObstacleState)
  (if (not (find-package "LOCAL_SENSING_NODE"))
    (make-package "LOCAL_SENSING_NODE"))
  (shadow 'DynamicObstacleState (find-package "LOCAL_SENSING_NODE")))
(unless (find-package "LOCAL_SENSING_NODE::DYNAMICOBSTACLESTATE")
  (make-package "LOCAL_SENSING_NODE::DYNAMICOBSTACLESTATE"))

(in-package "ROS")
;;//! \htmlinclude DynamicObstacleState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass local_sensing_node::DynamicObstacleState
  :super ros::object
  :slots (_obstacle_id _is_dynamic _geometry_type _position _velocity _size ))

(defmethod local_sensing_node::DynamicObstacleState
  (:init
   (&key
    ((:obstacle_id __obstacle_id) 0)
    ((:is_dynamic __is_dynamic) nil)
    ((:geometry_type __geometry_type) "")
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:size __size) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _obstacle_id (round __obstacle_id))
   (setq _is_dynamic __is_dynamic)
   (setq _geometry_type (string __geometry_type))
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _size __size)
   self)
  (:obstacle_id
   (&optional __obstacle_id)
   (if __obstacle_id (setq _obstacle_id __obstacle_id)) _obstacle_id)
  (:is_dynamic
   (&optional (__is_dynamic :null))
   (if (not (eq __is_dynamic :null)) (setq _is_dynamic __is_dynamic)) _is_dynamic)
  (:geometry_type
   (&optional __geometry_type)
   (if __geometry_type (setq _geometry_type __geometry_type)) _geometry_type)
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
    ;; int32 _obstacle_id
    4
    ;; bool _is_dynamic
    1
    ;; string _geometry_type
    4 (length _geometry_type)
    ;; geometry_msgs/Point _position
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
     ;; int32 _obstacle_id
       (write-long _obstacle_id s)
     ;; bool _is_dynamic
       (if _is_dynamic (write-byte -1 s) (write-byte 0 s))
     ;; string _geometry_type
       (write-long (length _geometry_type) s) (princ _geometry_type s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; geometry_msgs/Vector3 _size
       (send _size :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _obstacle_id
     (setq _obstacle_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _is_dynamic
     (setq _is_dynamic (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _geometry_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _geometry_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; geometry_msgs/Vector3 _size
     (send _size :deserialize buf ptr-) (incf ptr- (send _size :serialization-length))
   ;;
   self)
  )

(setf (get local_sensing_node::DynamicObstacleState :md5sum-) "d1cd6ca51f05588daa1bdc717cdb87ba")
(setf (get local_sensing_node::DynamicObstacleState :datatype-) "local_sensing_node/DynamicObstacleState")
(setf (get local_sensing_node::DynamicObstacleState :definition-)
      "int32 obstacle_id
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



(provide :local_sensing_node/DynamicObstacleState "d1cd6ca51f05588daa1bdc717cdb87ba")


