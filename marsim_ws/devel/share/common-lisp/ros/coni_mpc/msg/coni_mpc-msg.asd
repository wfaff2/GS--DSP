
(cl:in-package :asdf)

(defsystem "coni_mpc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LocalBarrier" :depends-on ("_package_LocalBarrier"))
    (:file "_package_LocalBarrier" :depends-on ("_package"))
  ))