
(cl:in-package :asdf)

(defsystem "local_sensing_node-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DynamicObstacleState" :depends-on ("_package_DynamicObstacleState"))
    (:file "_package_DynamicObstacleState" :depends-on ("_package"))
    (:file "DynamicObstacleStateArray" :depends-on ("_package_DynamicObstacleStateArray"))
    (:file "_package_DynamicObstacleStateArray" :depends-on ("_package"))
  ))