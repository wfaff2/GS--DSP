
(cl:in-package :asdf)

(defsystem "onboard_detector-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TrackedObstacle" :depends-on ("_package_TrackedObstacle"))
    (:file "_package_TrackedObstacle" :depends-on ("_package"))
    (:file "TrackedObstacleArray" :depends-on ("_package_TrackedObstacleArray"))
    (:file "_package_TrackedObstacleArray" :depends-on ("_package"))
  ))